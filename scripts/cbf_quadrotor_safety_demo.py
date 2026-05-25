"""
Minimum runnable demo for CBF-based safety-critical quadrotor position control.

The model is the translational outer-loop abstraction of a quadrotor:

    p_dot = v
    v_dot = u + d(t)

where p is position, v is velocity, u is the commanded acceleration, and d(t)
is an unknown wind-like disturbance. A robust PD nominal controller tracks a
reference trajectory. A high-order CBF-QP safety filter modifies the command
only when obstacle avoidance constraints are close to violation.

This is intentionally a compact research prototype, not a full 6-DOF flight
stack. It is the right first step before extending the method to attitude
dynamics and thrust/torque allocation.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "output" / "cbf_uav_demo"


@dataclass(frozen=True)
class Obstacle:
    center: np.ndarray
    radius: float
    label: str


@dataclass(frozen=True)
class ControllerConfig:
    name: str
    use_cbf: bool
    use_observer: bool
    use_robust: bool


def reference(t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Smooth 3D reference trajectory and its first two derivatives."""
    w = 0.22
    p = np.array(
        [
            -3.0 + 0.62 * t,
            1.2 * np.sin(w * t),
            1.2 + 0.35 * np.sin(0.5 * w * t),
        ]
    )
    v = np.array(
        [
            0.62,
            1.2 * w * np.cos(w * t),
            0.35 * 0.5 * w * np.cos(0.5 * w * t),
        ]
    )
    a = np.array(
        [
            0.0,
            -1.2 * w**2 * np.sin(w * t),
            -0.35 * (0.5 * w) ** 2 * np.sin(0.5 * w * t),
        ]
    )
    return p, v, a


def disturbance(t: float, p: np.ndarray) -> np.ndarray:
    """Bounded wind-like disturbance with a weak position-dependent component."""
    return np.array(
        [
            0.20 * np.sin(0.8 * t) + 0.06 * np.cos(p[1]),
            -0.18 * np.cos(0.55 * t),
            0.12 * np.sin(0.45 * t + 0.4),
        ]
    )


def nominal_control(
    t: float,
    p: np.ndarray,
    v: np.ndarray,
    d_hat: np.ndarray,
    config: ControllerConfig,
) -> np.ndarray:
    p_ref, v_ref, a_ref = reference(t)
    e_p = p - p_ref
    e_v = v - v_ref

    kp = np.array([1.8, 1.8, 2.2])
    kv = np.array([2.4, 2.4, 2.8])
    u = a_ref - kp * e_p - kv * e_v

    if config.use_observer:
        u = u - d_hat

    if config.use_robust:
        s = e_v + 0.9 * e_p
        u = u - 0.18 * np.tanh(4.0 * s)

    return u


def cbf_values(
    p: np.ndarray,
    v: np.ndarray,
    obstacles: list[Obstacle],
) -> tuple[np.ndarray, np.ndarray]:
    hs = []
    hdots = []
    for obstacle in obstacles:
        rel = p - obstacle.center
        h = float(rel @ rel - obstacle.radius**2)
        h_dot = float(2.0 * rel @ v)
        hs.append(h)
        hdots.append(h_dot)
    return np.array(hs), np.array(hdots)


def safety_filter(
    u_nom: np.ndarray,
    p: np.ndarray,
    v: np.ndarray,
    d_hat: np.ndarray,
    obstacles: list[Obstacle],
    u_min: np.ndarray,
    u_max: np.ndarray,
) -> np.ndarray:
    """Solve a small CBF-QP: minimize ||u-u_nom||^2 under safety and bounds."""
    gamma_0 = 1.6
    gamma_1 = 2.4
    constraints = []

    for obstacle in obstacles:
        rel = p - obstacle.center
        h = float(rel @ rel - obstacle.radius**2)
        h_dot = float(2.0 * rel @ v)

        # h_ddot = 2 v^T v + 2 rel^T (u + d).
        # Using d_hat makes the filter less conservative when the observer is on.
        a = 2.0 * rel
        b = -2.0 * float(v @ v) - 2.0 * float(rel @ d_hat) - gamma_1 * h_dot - gamma_0 * h

        constraints.append(
            {
                "type": "ineq",
                "fun": lambda u, a=a, b=b: float(a @ u - b),
            }
        )

    bounds = list(zip(u_min, u_max))
    x0 = np.clip(u_nom, u_min, u_max)

    result = minimize(
        lambda u: 0.5 * float((u - u_nom) @ (u - u_nom)),
        x0,
        jac=lambda u: u - u_nom,
        bounds=bounds,
        constraints=constraints,
        method="SLSQP",
        options={"ftol": 1e-9, "maxiter": 80, "disp": False},
    )

    if result.success:
        return np.asarray(result.x)

    # Fallback: keep the simulation alive and make failures visible in plots.
    return x0


def simulate(config: ControllerConfig) -> dict[str, np.ndarray]:
    dt = 0.02
    t_final = 22.0
    t_grid = np.arange(0.0, t_final + dt, dt)
    n = len(t_grid)

    obstacles = [
        Obstacle(center=np.array([1.0, 0.15, 1.22]), radius=0.92, label="O1"),
        Obstacle(center=np.array([5.2, -0.35, 1.05]), radius=0.72, label="O2"),
    ]

    u_min = np.array([-2.8, -2.8, -2.2])
    u_max = np.array([2.8, 2.8, 2.2])

    p = np.array([-3.0, 0.0, 1.0])
    v = np.array([0.45, 0.1, 0.0])
    d_hat = np.zeros(3)
    observer_gain = 3.5

    p_hist = np.zeros((n, 3))
    v_hist = np.zeros((n, 3))
    pref_hist = np.zeros((n, 3))
    u_hist = np.zeros((n, 3))
    d_hist = np.zeros((n, 3))
    dhat_hist = np.zeros((n, 3))
    h_hist = np.zeros((n, len(obstacles)))
    qp_delta_hist = np.zeros(n)

    measured_accel_prev = np.zeros(3)
    u_prev = np.zeros(3)

    for k, t in enumerate(t_grid):
        p_ref, _, _ = reference(t)
        d = disturbance(t, p)

        if config.use_observer:
            innovation = measured_accel_prev - u_prev - d_hat
            d_hat = d_hat + dt * observer_gain * innovation
        else:
            d_hat = np.zeros(3)

        u_nom = nominal_control(t, p, v, d_hat, config)
        if config.use_cbf:
            u = safety_filter(u_nom, p, v, d_hat, obstacles, u_min, u_max)
        else:
            u = np.clip(u_nom, u_min, u_max)

        accel = u + d

        p_hist[k] = p
        v_hist[k] = v
        pref_hist[k] = p_ref
        u_hist[k] = u
        d_hist[k] = d
        dhat_hist[k] = d_hat
        h_hist[k], _ = cbf_values(p, v, obstacles)
        qp_delta_hist[k] = float(np.linalg.norm(u - np.clip(u_nom, u_min, u_max)))

        # Semi-implicit Euler integration is stable enough for this prototype.
        v = v + dt * accel
        p = p + dt * v
        measured_accel_prev = accel
        u_prev = u

    return {
        "t": t_grid,
        "p": p_hist,
        "v": v_hist,
        "p_ref": pref_hist,
        "u": u_hist,
        "d": d_hist,
        "d_hat": dhat_hist,
        "h": h_hist,
        "qp_delta": qp_delta_hist,
        "obstacles": obstacles,
        "u_min": u_min,
        "u_max": u_max,
    }


def plot_results(results: dict[str, dict[str, np.ndarray]]) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    full = results["cbf_observer_robust"]
    obstacles: list[Obstacle] = full["obstacles"]

    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(full["p_ref"][:, 0], full["p_ref"][:, 1], full["p_ref"][:, 2], "k--", label="reference")
    for name, data in results.items():
        ax.plot(data["p"][:, 0], data["p"][:, 1], data["p"][:, 2], label=name)
    for obstacle in obstacles:
        u = np.linspace(0, 2 * np.pi, 32)
        vv = np.linspace(0, np.pi, 16)
        xs = obstacle.center[0] + obstacle.radius * np.outer(np.cos(u), np.sin(vv))
        ys = obstacle.center[1] + obstacle.radius * np.outer(np.sin(u), np.sin(vv))
        zs = obstacle.center[2] + obstacle.radius * np.outer(np.ones_like(u), np.cos(vv))
        ax.plot_surface(xs, ys, zs, color="tab:red", alpha=0.18, linewidth=0)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("3D trajectory with obstacle safety constraints")
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "trajectory_3d.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.8))
    for name, data in results.items():
        err = np.linalg.norm(data["p"] - data["p_ref"], axis=1)
        ax.plot(data["t"], err, label=name)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position error norm [m]")
    ax.set_title("Tracking error")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "tracking_error.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(len(obstacles), 1, figsize=(9, 5.4), sharex=True)
    if len(obstacles) == 1:
        axes = [axes]
    for idx, ax in enumerate(axes):
        for name, data in results.items():
            ax.plot(data["t"], data["h"][:, idx], label=name)
        ax.axhline(0.0, color="k", linestyle="--", linewidth=1.0)
        ax.set_ylabel(f"h_{idx + 1}(t)")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time [s]")
    axes[0].set_title("CBF safety functions, safe if h(t) >= 0")
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "safety_function.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(9, 6.2), sharex=True)
    labels = ["u_x", "u_y", "u_z"]
    for i, ax in enumerate(axes):
        ax.plot(full["t"], full["u"][:, i], label=labels[i])
        ax.axhline(full["u_min"][i], color="k", linestyle="--", linewidth=0.9)
        ax.axhline(full["u_max"][i], color="k", linestyle="--", linewidth=0.9)
        ax.set_ylabel(labels[i])
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time [s]")
    axes[0].set_title("Control input with saturation bounds")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "control_inputs.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(9, 6.2), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(full["t"], full["d"][:, i], "k-", label="true disturbance")
        ax.plot(full["t"], full["d_hat"][:, i], "r--", label="estimated")
        ax.set_ylabel(f"d_{i + 1}")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time [s]")
    axes[0].set_title("Disturbance observer")
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "disturbance_estimate.png", dpi=180)
    plt.close(fig)


def write_summary(results: dict[str, dict[str, np.ndarray]]) -> None:
    lines = ["case,min_h,final_error,mean_error,max_input_norm,mean_qp_delta"]
    for name, data in results.items():
        err = np.linalg.norm(data["p"] - data["p_ref"], axis=1)
        u_norm = np.linalg.norm(data["u"], axis=1)
        lines.append(
            ",".join(
                [
                    name,
                    f"{np.min(data['h']):.6f}",
                    f"{err[-1]:.6f}",
                    f"{np.mean(err):.6f}",
                    f"{np.max(u_norm):.6f}",
                    f"{np.mean(data['qp_delta']):.6f}",
                ]
            )
        )
    (OUT_DIR / "summary.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    configs = [
        ControllerConfig("baseline_no_cbf", use_cbf=False, use_observer=False, use_robust=True),
        ControllerConfig("cbf_no_observer", use_cbf=True, use_observer=False, use_robust=True),
        ControllerConfig("cbf_observer_robust", use_cbf=True, use_observer=True, use_robust=True),
    ]
    results = {config.name: simulate(config) for config in configs}
    plot_results(results)
    write_summary(results)

    summary = (OUT_DIR / "summary.csv").read_text(encoding="utf-8")
    print(f"Results written to: {OUT_DIR}")
    print(summary)


if __name__ == "__main__":
    main()
