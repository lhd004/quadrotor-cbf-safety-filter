from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from cbf_quadrotor_6dof_demo import ControllerConfig, ScenarioConfig, simulate


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "output" / "6dof_batch"
FIG_DIR = OUT_DIR / "figures"


plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans", "sans-serif"],
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "font.size": 8,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "axes.linewidth": 0.8,
        "legend.frameon": False,
    }
)


CONTROLLERS = [
    ControllerConfig("baseline", use_cbf=False, use_observer=False, use_robust=False),
    ControllerConfig("apf_avoidance", use_cbf=False, use_observer=False, use_robust=False, use_apf=True),
    ControllerConfig("predictive_safety_filter", use_cbf=False, use_observer=False, use_robust=False, use_predictive_filter=True),
    ControllerConfig("cbf_only", use_cbf=True, use_observer=False, use_robust=False),
    ControllerConfig("cbf_robust", use_cbf=True, use_observer=False, use_robust=True),
    ControllerConfig("cbf_observer", use_cbf=True, use_observer=True, use_robust=False),
    ControllerConfig("cbf_observer_robust", use_cbf=True, use_observer=True, use_robust=True),
]


SCENARIOS = [
    ScenarioConfig("nominal", disturbance_scale=1.0, obstacle_radius_scale=1.0, accel_limit_scale=1.0),
    ScenarioConfig("strong_disturbance", disturbance_scale=1.6, obstacle_radius_scale=1.0, accel_limit_scale=1.0),
    ScenarioConfig("large_obstacle", disturbance_scale=1.0, obstacle_radius_scale=1.12, accel_limit_scale=1.0),
    ScenarioConfig("tight_actuation", disturbance_scale=1.0, obstacle_radius_scale=1.0, accel_limit_scale=0.78, thrust_limit_scale=0.68),
    ScenarioConfig("narrow_passage", disturbance_scale=1.25, obstacle_radius_scale=1.08, accel_limit_scale=0.86, thrust_limit_scale=0.74),
]


def metrics(data: dict[str, np.ndarray]) -> dict[str, float]:
    err = np.linalg.norm(data["p"] - data["p_ref"], axis=1)
    e_r_norm = np.linalg.norm(data["e_r"], axis=1)
    thrust = data["thrust"]
    params = data["params"]
    return {
        "min_h": float(np.min(data["h"])),
        "final_error": float(err[-1]),
        "mean_error": float(np.mean(err)),
        "max_error": float(np.max(err)),
        "max_attitude_error_deg": float(np.rad2deg(np.max(e_r_norm))),
        "max_thrust": float(np.max(thrust)),
        "thrust_saturation_ratio": float(np.mean(thrust >= 0.995 * params.thrust_max)),
    }


def observer_metrics(data: dict[str, np.ndarray]) -> dict[str, float]:
    error = data["d"] - data["d_hat"]
    error_norm = np.linalg.norm(error, axis=1)
    disturbance_norm = np.linalg.norm(data["d"], axis=1)
    return {
        "mean_observer_error": float(np.mean(error_norm)),
        "max_observer_error": float(np.max(error_norm)),
        "mean_disturbance_norm": float(np.mean(disturbance_norm)),
        "final_observer_error": float(error_norm[-1]),
    }


def safety_filter_metrics(data: dict[str, np.ndarray]) -> dict[str, float]:
    correction = data["correction_norm"]
    min_cbf_slack = data["min_cbf_slack"]
    finite_slack = min_cbf_slack[np.isfinite(min_cbf_slack)]
    active = correction > 1e-4
    return {
        "qp_success_rate": float(np.mean(data["qp_success"])),
        "active_rate": float(np.mean(active)),
        "mean_correction": float(np.mean(correction)),
        "max_correction": float(np.max(correction)),
        "min_cbf_slack": float(np.min(finite_slack)) if finite_slack.size else float("nan"),
    }


def write_observer_summary(results: dict[tuple[str, str], dict[str, np.ndarray]]) -> None:
    rows: list[dict[str, object]] = []
    for scenario in SCENARIOS:
        key = (scenario.name, "cbf_observer_robust")
        if key not in results:
            continue
        row: dict[str, object] = {"scenario": scenario.name, "controller": "cbf_observer_robust"}
        row.update({name: f"{value:.6f}" for name, value in observer_metrics(results[key]).items()})
        rows.append(row)

    fieldnames = [
        "scenario",
        "controller",
        "mean_observer_error",
        "max_observer_error",
        "mean_disturbance_norm",
        "final_observer_error",
    ]
    with (OUT_DIR / "observer_summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_safety_filter_summary(results: dict[tuple[str, str], dict[str, np.ndarray]]) -> None:
    rows: list[dict[str, object]] = []
    cbf_controllers = [controller for controller in CONTROLLERS if controller.use_cbf]
    for scenario in SCENARIOS:
        for controller in cbf_controllers:
            key = (scenario.name, controller.name)
            if key not in results:
                continue
            row: dict[str, object] = {"scenario": scenario.name, "controller": controller.name}
            row.update({name: f"{value:.6f}" for name, value in safety_filter_metrics(results[key]).items()})
            rows.append(row)

    fieldnames = [
        "scenario",
        "controller",
        "qp_success_rate",
        "active_rate",
        "mean_correction",
        "max_correction",
        "min_cbf_slack",
    ]
    with (OUT_DIR / "safety_filter_diagnostics.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def save_publication_figure(fig: plt.Figure, stem: Path) -> None:
    fig.savefig(stem.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(stem.with_suffix(".pdf"), bbox_inches="tight")
    fig.savefig(stem.with_suffix(".svg"), bbox_inches="tight")
    fig.savefig(stem.with_suffix(".tiff"), dpi=600, bbox_inches="tight")


def write_summary(rows: list[dict[str, object]]) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "scenario",
        "controller",
        "min_h",
        "final_error",
        "mean_error",
        "max_error",
        "max_attitude_error_deg",
        "max_thrust",
        "thrust_saturation_ratio",
    ]
    with (OUT_DIR / "summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_latex_table(rows: list[dict[str, object]]) -> None:
    lines = [
        r"\begin{table}[t]",
        r"\caption{Batch 6-DOF simulation, ablation, and baseline-comparison results. Positive $\min h$ indicates that all obstacle constraints are respected. Bold values mark the best safety margin or tracking error within each scenario.}",
        r"\label{tab:batch_6dof}",
        r"\centering",
        r"\scriptsize",
        r"\resizebox{\linewidth}{!}{%",
        r"\begin{tabular}{llrrrr}",
        r"\toprule",
        r"Scenario & Controller & $\min h \uparrow$ & Final err. $\downarrow$ & Mean err. $\downarrow$ & Max thrust \\",
        r"\midrule",
    ]
    best_by_scenario: dict[str, dict[str, float]] = {}
    for scenario in {str(row["scenario"]) for row in rows}:
        scenario_rows = [row for row in rows if row["scenario"] == scenario]
        best_by_scenario[scenario] = {
            "min_h": max(float(row["min_h"]) for row in scenario_rows),
            "final_error": min(float(row["final_error"]) for row in scenario_rows),
            "mean_error": min(float(row["mean_error"]) for row in scenario_rows),
        }

    def maybe_bold(value: float, best_value: float) -> str:
        text = f"{value:.3f}"
        if abs(value - best_value) < 5e-4:
            return rf"\textbf{{{text}}}"
        return text

    for row in rows:
        scenario = str(row["scenario"])
        controller = str(row["controller"])
        best = best_by_scenario[scenario]
        min_h = maybe_bold(float(row["min_h"]), best["min_h"])
        final_error = maybe_bold(float(row["final_error"]), best["final_error"])
        mean_error = maybe_bold(float(row["mean_error"]), best["mean_error"])
        max_thrust = f"{float(row['max_thrust']):.2f}"
        scenario_name = str(row["scenario"]).replace("_", r"\_")
        controller_name = controller.replace("_", r"\_")
        lines.append(f"{scenario_name} & {controller_name} & {min_h} & {final_error} & {mean_error} & {max_thrust} \\\\")
    lines.extend([r"\bottomrule", r"\end{tabular}%", r"}", r"\end{table}", ""])
    (OUT_DIR / "summary_table.tex").write_text("\n".join(lines), encoding="utf-8")


def plot_main(results: dict[tuple[str, str], dict[str, np.ndarray]]) -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    scenario = "nominal"
    fig = plt.figure(figsize=(9.5, 6.2), dpi=180)
    ax = fig.add_subplot(111, projection="3d")
    ref_data = results[(scenario, "cbf_observer_robust")]
    ax.plot(ref_data["p_ref"][:, 0], ref_data["p_ref"][:, 1], ref_data["p_ref"][:, 2], "k--", label="reference")
    for controller in CONTROLLERS:
        data = results[(scenario, controller.name)]
        ax.plot(data["p"][:, 0], data["p"][:, 1], data["p"][:, 2], label=controller.name)
    for obstacle in ref_data["obstacles"]:
        u = np.linspace(0, 2 * np.pi, 36)
        v = np.linspace(0, np.pi, 18)
        xs = obstacle.center[0] + obstacle.radius * np.outer(np.cos(u), np.sin(v))
        ys = obstacle.center[1] + obstacle.radius * np.outer(np.sin(u), np.sin(v))
        zs = obstacle.center[2] + obstacle.radius * np.outer(np.ones_like(u), np.cos(v))
        ax.plot_surface(xs, ys, zs, color="tab:red", alpha=0.16, linewidth=0)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("Nominal 6-DOF trajectory comparison")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "main_trajectory_3d.png")
    plt.close(fig)


def plot_metric_bars(rows: list[dict[str, object]]) -> None:
    scenarios = [scenario.name for scenario in SCENARIOS]
    controllers = [controller.name for controller in CONTROLLERS]
    x = np.arange(len(scenarios))
    width = 0.12

    for metric_name, ylabel, filename, zero_line in [
        ("min_h", "minimum barrier value", "safety_by_scenario.png", True),
        ("mean_error", "mean position error [m]", "tracking_by_scenario.png", False),
    ]:
        fig, ax = plt.subplots(figsize=(10.5, 4.8), dpi=180)
        for idx, controller in enumerate(controllers):
            values = [
                float(next(row for row in rows if row["scenario"] == scenario and row["controller"] == controller)[metric_name])
                for scenario in scenarios
            ]
            offset = (idx - (len(controllers) - 1) / 2.0) * width
            ax.bar(x + offset, values, width, label=controller)
        if zero_line:
            ax.axhline(0.0, color="k", linestyle="--", linewidth=1.0)
        ax.set_xticks(x)
        ax.set_xticklabels(scenarios, rotation=18, ha="right")
        ax.set_ylabel(ylabel)
        ax.grid(True, axis="y", alpha=0.3)
        ax.legend(fontsize=7, ncols=2)
        fig.tight_layout()
        fig.savefig(FIG_DIR / filename)
        plt.close(fig)


def plot_nominal_safety(results: dict[tuple[str, str], dict[str, np.ndarray]]) -> None:
    fig, ax = plt.subplots(figsize=(9.5, 4.6), dpi=180)
    for controller in CONTROLLERS:
        data = results[("nominal", controller.name)]
        ax.plot(data["t"], np.min(data["h"], axis=1), label=controller.name)
    ax.axhline(0.0, color="k", linestyle="--", linewidth=1.0)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("minimum h(t)")
    ax.set_title("Nominal scenario safety function")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "nominal_safety_function.png")
    plt.close(fig)


def plot_observer_diagnostics(results: dict[tuple[str, str], dict[str, np.ndarray]]) -> None:
    scenario = "strong_disturbance"
    controller = "cbf_observer_robust"
    data = results[(scenario, controller)]
    t = data["t"]
    error_norm = np.linalg.norm(data["d"] - data["d_hat"], axis=1)
    labels = ["x", "y", "z"]
    colors = ["#2A6FBB", "#5A9E6F", "#9B5B9E"]

    fig, axes = plt.subplots(2, 1, figsize=(7.2, 5.4), dpi=220, sharex=True, gridspec_kw={"height_ratios": [2.2, 1.0]})
    for idx, label in enumerate(labels):
        axes[0].plot(t, data["d"][:, idx], color=colors[idx], linewidth=1.7, label=rf"true $d_{label}$")
        axes[0].plot(t, data["d_hat"][:, idx], color=colors[idx], linestyle="--", linewidth=1.5, label=rf"estimated $\hat d_{label}$")
    axes[0].set_ylabel("disturbance [m s$^{-2}$]")
    axes[0].set_title("Disturbance-observer diagnostic under strong disturbance", loc="left", fontsize=8.5)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(ncols=3, fontsize=6.4, handlelength=2.3)

    axes[1].plot(t, error_norm, color="#A83232", linewidth=1.8)
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel(r"$\|d-\hat d\|$")
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    save_publication_figure(fig, FIG_DIR / "disturbance_observer_diagnostic")
    plt.close(fig)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    FIG_DIR.mkdir(parents=True, exist_ok=True)

    results: dict[tuple[str, str], dict[str, np.ndarray]] = {}
    rows: list[dict[str, object]] = []

    for scenario in SCENARIOS:
        for controller in CONTROLLERS:
            print(f"Running scenario={scenario.name}, controller={controller.name}")
            data = simulate(controller, scenario)
            results[(scenario.name, controller.name)] = data
            row = {"scenario": scenario.name, "controller": controller.name}
            row.update({key: f"{value:.6f}" for key, value in metrics(data).items()})
            rows.append(row)

    write_summary(rows)
    write_latex_table(rows)
    write_observer_summary(results)
    write_safety_filter_summary(results)
    plot_main(results)
    plot_metric_bars(rows)
    plot_nominal_safety(results)
    plot_observer_diagnostics(results)

    print(f"Results written to {OUT_DIR}")
    print((OUT_DIR / "summary.csv").read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
