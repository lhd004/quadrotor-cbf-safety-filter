"""
6-DOF quadrotor demo for CBF-based safety-critical trajectory tracking.

This script extends the first-stage translational model into a full quadrotor
rigid-body simulation:

    m p_ddot = T R e3 - m g e3 + m d_p(t)
    J omega_dot = tau - omega x J omega + d_R(t)
    R_dot = R hat(omega)

The outer loop computes a safe desired translational acceleration with a
high-order CBF-QP. The command is converted into total thrust and desired
attitude. The inner loop uses a geometric SO(3) attitude controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "output" / "cbf_uav_6dof"
E3 = np.array([0.0, 0.0, 1.0])


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
    use_apf: bool = False
    use_predictive_filter: bool = False


@dataclass(frozen=True)
class ScenarioConfig:
    name: str
    disturbance_scale: float = 1.0
    obstacle_radius_scale: float = 1.0
    accel_limit_scale: float = 1.0
    lateral_reference_scale: float = 1.0
    thrust_limit_scale: float = 1.0
    cbf_gain_scale: float = 1.0


@dataclass(frozen=True)
class QuadrotorParams:
    mass: float
    gravity: float
    inertia: np.ndarray
    thrust_min: float
    thrust_max: float
    torque_limit: np.ndarray


def hat(x: np.ndarray) -> np.ndarray:
    return np.array(
        [
            [0.0, -x[2], x[1]],
            [x[2], 0.0, -x[0]],
            [-x[1], x[0], 0.0],
        ]
    )


def vee(x_hat: np.ndarray) -> np.ndarray:
    return np.array([x_hat[2, 1], x_hat[0, 2], x_hat[1, 0]])


def normalize(x: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(x)
    if norm < 1e-9:
        return fallback.copy()
    return x / norm


def project_to_so3(r: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(r)
    projected = u @ vt
    if np.linalg.det(projected) < 0.0:
        u[:, -1] *= -1.0
        projected = u @ vt
    return projected


def reference(t: float, lateral_scale: float = 1.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    w = 0.20
    p = np.array(
        [
            -3.0 + 0.52 * t,
            lateral_scale * 1.05 * np.sin(w * t),
            1.25 + 0.28 * np.sin(0.5 * w * t),
        ]
    )
    v = np.array(
        [
            0.52,
            lateral_scale * 1.05 * w * np.cos(w * t),
            0.28 * 0.5 * w * np.cos(0.5 * w * t),
        ]
    )
    a = np.array(
        [
            0.0,
            -lateral_scale * 1.05 * w**2 * np.sin(w * t),
            -0.28 * (0.5 * w) ** 2 * np.sin(0.5 * w * t),
        ]
    )
    return p, v, a


def translational_disturbance(t: float, p: np.ndarray, scale: float = 1.0) -> np.ndarray:
    return scale * np.array(
        [
            0.16 * np.sin(0.75 * t) + 0.04 * np.cos(p[1]),
            -0.14 * np.cos(0.50 * t),
            0.08 * np.sin(0.42 * t + 0.3),
        ]
    )


def rotational_disturbance(t: float) -> np.ndarray:
    return np.array(
        [
            0.0020 * np.sin(0.9 * t),
            -0.0016 * np.cos(0.7 * t),
            0.0012 * np.sin(0.5 * t + 0.2),
        ]
    )


def desired_attitude(accel_cmd: np.ndarray, yaw: float, params: QuadrotorParams) -> tuple[np.ndarray, float]:
    desired_force = accel_cmd + params.gravity * E3
    b3_des = normalize(desired_force, E3)

    b1_yaw = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    b2_des = normalize(np.cross(b3_des, b1_yaw), np.array([0.0, 1.0, 0.0]))
    b1_des = np.cross(b2_des, b3_des)
    r_des = np.column_stack((b1_des, b2_des, b3_des))
    thrust = params.mass * float(desired_force @ b3_des)
    thrust = float(np.clip(thrust, params.thrust_min, params.thrust_max))
    return r_des, thrust


def nominal_acceleration(
    t: float,
    p: np.ndarray,
    v: np.ndarray,
    d_hat: np.ndarray,
    config: ControllerConfig,
    scenario: ScenarioConfig,
    obstacles: list[Obstacle],
) -> np.ndarray:
    p_ref, v_ref, a_ref = reference(t, scenario.lateral_reference_scale)
    e_p = p - p_ref
    e_v = v - v_ref

    kp = np.array([1.35, 1.35, 1.65])
    kv = np.array([2.05, 2.05, 2.25])
    accel = a_ref - kp * e_p - kv * e_v

    if config.use_observer:
        accel = accel - d_hat

    if config.use_robust:
        s = e_v + 0.8 * e_p
        accel = accel - 0.12 * np.tanh(4.0 * s)

    if config.use_apf:
        accel = accel + apf_avoidance_acceleration(p, obstacles)

    return accel


def apf_avoidance_acceleration(p: np.ndarray, obstacles: list[Obstacle]) -> np.ndarray:
    repulsive = np.zeros(3)
    influence_margin = 1.25
    gain = 0.32
    max_norm = 1.35
    for obstacle in obstacles:
        rel = p - obstacle.center
        distance = max(float(np.linalg.norm(rel)), 1e-6)
        influence_distance = obstacle.radius + influence_margin
        if distance >= influence_distance:
            continue
        direction = rel / distance
        strength = gain * (1.0 / distance - 1.0 / influence_distance) / (distance**2)
        repulsive += strength * direction
    norm = float(np.linalg.norm(repulsive))
    if norm > max_norm:
        repulsive = max_norm * repulsive / norm
    return repulsive


def cbf_values(p: np.ndarray, v: np.ndarray, obstacles: list[Obstacle]) -> np.ndarray:
    return np.array([float((p - obs.center) @ (p - obs.center) - obs.radius**2) for obs in obstacles])


def safety_filter(
    accel_nom: np.ndarray,
    p: np.ndarray,
    v: np.ndarray,
    d_hat: np.ndarray,
    obstacles: list[Obstacle],
    accel_min: np.ndarray,
    accel_max: np.ndarray,
    gamma_scale: float = 1.0,
) -> tuple[np.ndarray, bool, float]:
    gamma_0 = 1.35 * gamma_scale
    gamma_1 = 2.25 * gamma_scale
    constraints = []
    constraint_rows: list[tuple[np.ndarray, float]] = []

    for obstacle in obstacles:
        rel = p - obstacle.center
        h = float(rel @ rel - obstacle.radius**2)
        h_dot = float(2.0 * rel @ v)
        a = 2.0 * rel
        b = -2.0 * float(v @ v) - 2.0 * float(rel @ d_hat) - gamma_1 * h_dot - gamma_0 * h
        constraint_rows.append((a, b))
        constraints.append({"type": "ineq", "fun": lambda u, a=a, b=b: float(a @ u - b)})

    x0 = np.clip(accel_nom, accel_min, accel_max)
    result = minimize(
        lambda u: 0.5 * float((u - accel_nom) @ (u - accel_nom)),
        x0,
        jac=lambda u: u - accel_nom,
        bounds=list(zip(accel_min, accel_max)),
        constraints=constraints,
        method="SLSQP",
        options={"ftol": 1e-9, "maxiter": 80, "disp": False},
    )
    if result.success:
        accel_safe = np.asarray(result.x)
        min_slack = min(float(a @ accel_safe - b) for a, b in constraint_rows)
        return accel_safe, True, min_slack
    min_slack = min(float(a @ x0 - b) for a, b in constraint_rows)
    return x0, False, min_slack


def predictive_safety_filter(
    accel_nom: np.ndarray,
    t: float,
    p: np.ndarray,
    v: np.ndarray,
    d_hat: np.ndarray,
    obstacles: list[Obstacle],
    scenario: ScenarioConfig,
    accel_min: np.ndarray,
    accel_max: np.ndarray,
) -> tuple[np.ndarray, bool, float]:
    horizon = np.array([0.12, 0.24, 0.36, 0.48, 0.60, 0.72])
    safety_margin = 0.015

    constraints = []
    constraint_rows: list[tuple[float, float]] = []
    for tau in horizon:
        for obstacle in obstacles:
            def predicted_h(u: np.ndarray, tau: float = tau, obstacle: Obstacle = obstacle) -> float:
                p_pred = p + tau * v + 0.5 * tau**2 * (u + d_hat)
                return float((p_pred - obstacle.center) @ (p_pred - obstacle.center) - obstacle.radius**2 - safety_margin)

            constraints.append({"type": "ineq", "fun": predicted_h})

    def objective(u: np.ndarray) -> float:
        total = 0.18 * float((u - accel_nom) @ (u - accel_nom))
        for tau in horizon:
            p_ref, v_ref, _ = reference(t + float(tau), scenario.lateral_reference_scale)
            p_pred = p + tau * v + 0.5 * tau**2 * (u + d_hat)
            v_pred = v + tau * (u + d_hat)
            total += 1.6 * float((p_pred - p_ref) @ (p_pred - p_ref))
            total += 0.35 * float((v_pred - v_ref) @ (v_pred - v_ref))
        return total

    x0 = np.clip(accel_nom, accel_min, accel_max)
    result = minimize(
        objective,
        x0,
        bounds=list(zip(accel_min, accel_max)),
        constraints=constraints,
        method="SLSQP",
        options={"ftol": 1e-7, "maxiter": 60, "disp": False},
    )

    accel_safe = np.asarray(result.x) if result.success else x0
    min_margin = float("inf")
    for tau in horizon:
        for obstacle in obstacles:
            p_pred = p + tau * v + 0.5 * tau**2 * (accel_safe + d_hat)
            margin = float((p_pred - obstacle.center) @ (p_pred - obstacle.center) - obstacle.radius**2 - safety_margin)
            min_margin = min(min_margin, margin)
    return accel_safe, bool(result.success), min_margin


def attitude_torque(
    r: np.ndarray,
    omega: np.ndarray,
    r_des: np.ndarray,
    params: QuadrotorParams,
) -> tuple[np.ndarray, np.ndarray]:
    e_r = 0.5 * vee(r_des.T @ r - r.T @ r_des)
    e_w = omega
    k_r = np.array([0.42, 0.42, 0.26])
    k_w = np.array([0.070, 0.070, 0.045])
    tau = -k_r * e_r - k_w * e_w + np.cross(omega, params.inertia @ omega)
    tau = np.clip(tau, -params.torque_limit, params.torque_limit)
    return tau, e_r


def make_obstacles(scenario: ScenarioConfig) -> list[Obstacle]:
    return [
        Obstacle(center=np.array([0.75, 0.12, 1.22]), radius=0.82 * scenario.obstacle_radius_scale, label="O1"),
        Obstacle(center=np.array([4.55, -0.32, 1.10]), radius=0.64 * scenario.obstacle_radius_scale, label="O2"),
    ]


def simulate(config: ControllerConfig, scenario: ScenarioConfig | None = None) -> dict[str, np.ndarray]:
    if scenario is None:
        scenario = ScenarioConfig("nominal")
    params = QuadrotorParams(
        mass=1.25,
        gravity=9.81,
        inertia=np.diag([0.018, 0.018, 0.032]),
        thrust_min=0.0,
        thrust_max=19.0 * scenario.thrust_limit_scale,
        torque_limit=np.array([0.22, 0.22, 0.12]),
    )
    inv_j = np.linalg.inv(params.inertia)

    dt = 0.005
    t_final = 22.0
    t_grid = np.arange(0.0, t_final + dt, dt)
    n = len(t_grid)

    obstacles = make_obstacles(scenario)
    accel_min = scenario.accel_limit_scale * np.array([-2.4, -2.4, -1.8])
    accel_max = scenario.accel_limit_scale * np.array([2.4, 2.4, 1.8])

    p = np.array([-3.0, 0.0, 1.0])
    v = np.array([0.35, 0.08, 0.0])
    r = np.eye(3)
    omega = np.zeros(3)
    d_hat = np.zeros(3)
    observer_gain = 2.8

    p_hist = np.zeros((n, 3))
    p_ref_hist = np.zeros((n, 3))
    u_hist = np.zeros((n, 3))
    u_nom_hist = np.zeros((n, 3))
    correction_norm_hist = np.zeros(n)
    qp_success_hist = np.ones(n)
    min_cbf_slack_hist = np.full(n, np.nan)
    thrust_hist = np.zeros(n)
    tau_hist = np.zeros((n, 3))
    d_hist = np.zeros((n, 3))
    d_hat_hist = np.zeros((n, 3))
    h_hist = np.zeros((n, len(obstacles)))
    e_r_hist = np.zeros((n, 3))
    roll_pitch_yaw_hist = np.zeros((n, 3))

    measured_accel_prev = np.zeros(3)
    model_accel_prev = np.zeros(3)
    predictive_update_steps = max(1, int(round(0.05 / dt)))
    predictive_accel_cmd = np.zeros(3)
    predictive_success = True
    predictive_min_margin = np.nan

    for k, t in enumerate(t_grid):
        p_ref, _, _ = reference(t, scenario.lateral_reference_scale)
        d_p = translational_disturbance(t, p, scenario.disturbance_scale)

        if config.use_observer:
            innovation = measured_accel_prev - model_accel_prev - d_hat
            d_hat = d_hat + dt * observer_gain * innovation
        else:
            d_hat = np.zeros(3)

        accel_nom = nominal_acceleration(t, p, v, d_hat, config, scenario, obstacles)
        if config.use_predictive_filter:
            if k % predictive_update_steps == 0:
                predictive_accel_cmd, predictive_success, predictive_min_margin = predictive_safety_filter(
                    accel_nom,
                    t,
                    p,
                    v,
                    d_hat,
                    obstacles,
                    scenario,
                    accel_min,
                    accel_max,
                )
            accel_cmd = np.clip(predictive_accel_cmd, accel_min, accel_max)
            qp_success = predictive_success
            min_cbf_slack = predictive_min_margin
        elif config.use_cbf:
            accel_cmd, qp_success, min_cbf_slack = safety_filter(
                accel_nom,
                p,
                v,
                d_hat,
                obstacles,
                accel_min,
                accel_max,
                scenario.cbf_gain_scale,
            )
        else:
            accel_cmd = np.clip(accel_nom, accel_min, accel_max)
            qp_success = True
            min_cbf_slack = np.nan

        r_des, thrust = desired_attitude(accel_cmd, yaw=0.0, params=params)
        tau, e_r = attitude_torque(r, omega, r_des, params)

        model_accel = (thrust / params.mass) * (r @ E3) - params.gravity * E3
        accel_actual = model_accel + d_p
        omega_dot = inv_j @ (tau - np.cross(omega, params.inertia @ omega) + rotational_disturbance(t))

        p_hist[k] = p
        p_ref_hist[k] = p_ref
        u_hist[k] = accel_cmd
        u_nom_hist[k] = accel_nom
        correction_norm_hist[k] = np.linalg.norm(accel_cmd - accel_nom)
        qp_success_hist[k] = float(qp_success)
        min_cbf_slack_hist[k] = min_cbf_slack
        thrust_hist[k] = thrust
        tau_hist[k] = tau
        d_hist[k] = d_p
        d_hat_hist[k] = d_hat
        h_hist[k] = cbf_values(p, v, obstacles)
        e_r_hist[k] = e_r
        roll_pitch_yaw_hist[k] = np.array(
            [
                np.arctan2(r[2, 1], r[2, 2]),
                np.arcsin(np.clip(-r[2, 0], -1.0, 1.0)),
                np.arctan2(r[1, 0], r[0, 0]),
            ]
        )

        v = v + dt * accel_actual
        p = p + dt * v
        omega = omega + dt * omega_dot
        r = project_to_so3(r + dt * r @ hat(omega))

        measured_accel_prev = accel_actual
        model_accel_prev = model_accel

    return {
        "t": t_grid,
        "p": p_hist,
        "p_ref": p_ref_hist,
        "u": u_hist,
        "u_nom": u_nom_hist,
        "correction_norm": correction_norm_hist,
        "qp_success": qp_success_hist,
        "min_cbf_slack": min_cbf_slack_hist,
        "thrust": thrust_hist,
        "tau": tau_hist,
        "d": d_hist,
        "d_hat": d_hat_hist,
        "h": h_hist,
        "e_r": e_r_hist,
        "rpy": roll_pitch_yaw_hist,
        "obstacles": obstacles,
        "params": params,
        "scenario": scenario,
    }


def plot_results(results: dict[str, dict[str, np.ndarray]]) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    full = results["full_cbf_observer_robust"]
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
    ax.set_title("6-DOF quadrotor trajectory")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "trajectory_3d.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.8))
    for name, data in results.items():
        err = np.linalg.norm(data["p"] - data["p_ref"], axis=1)
        ax.plot(data["t"], err, label=name)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position error norm [m]")
    ax.set_title("6-DOF tracking error")
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
    axes[0].set_title("6-DOF CBF safety functions")
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "safety_function.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(9, 7.2), sharex=True)
    axes[0].plot(full["t"], full["thrust"])
    axes[0].axhline(full["params"].thrust_min, color="k", linestyle="--", linewidth=0.9)
    axes[0].axhline(full["params"].thrust_max, color="k", linestyle="--", linewidth=0.9)
    axes[0].set_ylabel("T [N]")
    for i, label in enumerate(["tau_x", "tau_y", "tau_z"]):
        axes[i + 1].plot(full["t"], full["tau"][:, i])
        axes[i + 1].axhline(full["params"].torque_limit[i], color="k", linestyle="--", linewidth=0.9)
        axes[i + 1].axhline(-full["params"].torque_limit[i], color="k", linestyle="--", linewidth=0.9)
        axes[i + 1].set_ylabel(label)
    axes[-1].set_xlabel("time [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    axes[0].set_title("Thrust and attitude torques")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "thrust_and_torque.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(9, 6.2), sharex=True)
    for i, label in enumerate(["roll", "pitch", "yaw"]):
        axes[i].plot(full["t"], np.rad2deg(full["rpy"][:, i]))
        axes[i].set_ylabel(f"{label} [deg]")
        axes[i].grid(True, alpha=0.3)
    axes[-1].set_xlabel("time [s]")
    axes[0].set_title("Attitude response")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "attitude_response.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(9, 6.2), sharex=True)
    for i in range(3):
        axes[i].plot(full["t"], full["d"][:, i], "k-", label="true disturbance")
        axes[i].plot(full["t"], full["d_hat"][:, i], "r--", label="estimated")
        axes[i].set_ylabel(f"d_{i + 1}")
        axes[i].grid(True, alpha=0.3)
    axes[-1].set_xlabel("time [s]")
    axes[0].set_title("Translational disturbance observer")
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT_DIR / "disturbance_estimate.png", dpi=180)
    plt.close(fig)


def write_summary(results: dict[str, dict[str, np.ndarray]]) -> None:
    lines = ["case,min_h,final_error,mean_error,max_attitude_error_deg,max_thrust"]
    for name, data in results.items():
        err = np.linalg.norm(data["p"] - data["p_ref"], axis=1)
        e_r_norm = np.linalg.norm(data["e_r"], axis=1)
        lines.append(
            ",".join(
                [
                    name,
                    f"{np.min(data['h']):.6f}",
                    f"{err[-1]:.6f}",
                    f"{np.mean(err):.6f}",
                    f"{np.rad2deg(np.max(e_r_norm)):.6f}",
                    f"{np.max(data['thrust']):.6f}",
                ]
            )
        )
    (OUT_DIR / "summary.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    configs = [
        ControllerConfig("full_baseline_no_cbf", use_cbf=False, use_observer=False, use_robust=True),
        ControllerConfig("full_cbf_no_observer", use_cbf=True, use_observer=False, use_robust=True),
        ControllerConfig("full_cbf_observer_robust", use_cbf=True, use_observer=True, use_robust=True),
    ]
    results = {config.name: simulate(config) for config in configs}
    plot_results(results)
    write_summary(results)

    print(f"Results written to: {OUT_DIR}")
    print((OUT_DIR / "summary.csv").read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
