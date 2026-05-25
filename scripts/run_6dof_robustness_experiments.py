from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from cbf_quadrotor_6dof_demo import ControllerConfig, ScenarioConfig, simulate
from run_6dof_batch_experiments import save_publication_figure


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "output" / "6dof_robustness"
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


FULL_CONTROLLER = ControllerConfig(
    "cbf_observer_robust",
    use_cbf=True,
    use_observer=True,
    use_robust=True,
)


def metrics(data: dict[str, np.ndarray]) -> dict[str, float]:
    error = np.linalg.norm(data["p"] - data["p_ref"], axis=1)
    correction = data["correction_norm"]
    active = correction > 1e-4
    thrust = data["thrust"]
    params = data["params"]
    return {
        "min_h": float(np.min(data["h"])),
        "mean_error": float(np.mean(error)),
        "max_error": float(np.max(error)),
        "qp_success_rate": float(np.mean(data["qp_success"])),
        "active_rate": float(np.mean(active)),
        "max_correction": float(np.max(correction)),
        "max_thrust": float(np.max(thrust)),
        "thrust_saturation_ratio": float(np.mean(thrust >= 0.995 * params.thrust_max)),
    }


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def format_row(kind: str, label: str, data: dict[str, np.ndarray]) -> dict[str, object]:
    values = metrics(data)
    row: dict[str, object] = {"experiment": kind, "case": label}
    row.update({name: f"{value:.6f}" for name, value in values.items()})
    return row


def run_sensitivity() -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for gain_scale in [0.65, 0.80, 1.00, 1.20, 1.45]:
        scenario = ScenarioConfig(
            name=f"cbf_gain_{gain_scale:.2f}",
            disturbance_scale=1.6,
            obstacle_radius_scale=1.0,
            accel_limit_scale=1.0,
            thrust_limit_scale=1.0,
            cbf_gain_scale=gain_scale,
        )
        print(f"Running CBF gain sensitivity: {gain_scale:.2f}")
        data = simulate(FULL_CONTROLLER, scenario)
        rows.append(format_row("cbf_gain_sensitivity", f"{gain_scale:.2f}", data))
    return rows


def run_actuation_stress() -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    cases = [
        ("nominal_limits", 1.00, 1.00),
        ("moderate_limits", 0.82, 0.76),
        ("tight_limits", 0.70, 0.66),
        ("severe_limits", 0.58, 0.58),
    ]
    for label, accel_scale, thrust_scale in cases:
        scenario = ScenarioConfig(
            name=label,
            disturbance_scale=1.35,
            obstacle_radius_scale=1.05,
            accel_limit_scale=accel_scale,
            thrust_limit_scale=thrust_scale,
            cbf_gain_scale=1.0,
        )
        print(f"Running actuation stress: {label}")
        data = simulate(FULL_CONTROLLER, scenario)
        rows.append(format_row("actuation_stress", label, data))
    return rows


def write_latex_table(rows: list[dict[str, object]]) -> None:
    lines = [
        r"\begin{table}[t]",
        r"\caption{Robustness checks for the full observer-assisted robust CBF controller. The CBF-gain sweep uses the strong-disturbance setting; the actuation-stress sweep tightens acceleration and thrust limits. Positive $\min h$ indicates safety-constraint satisfaction in the tested run.}",
        r"\label{tab:robustness_checks}",
        r"\centering",
        r"\scriptsize",
        r"\resizebox{\linewidth}{!}{%",
        r"\begin{tabular}{llrrrrr}",
        r"\toprule",
        r"Experiment & Case & $\min h$ & Mean err. & QP success & Active rate & Thrust sat. \\",
        r"\midrule",
    ]
    for row in rows:
        experiment = str(row["experiment"]).replace("_", r"\_")
        case = str(row["case"]).replace("_", r"\_")
        lines.append(
            f"{experiment} & {case} & "
            f"{float(row['min_h']):.3f} & "
            f"{float(row['mean_error']):.3f} & "
            f"{float(row['qp_success_rate']):.3f} & "
            f"{float(row['active_rate']):.3f} & "
            f"{float(row['thrust_saturation_ratio']):.3f} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabular}%", r"}", r"\end{table}", ""])
    (OUT_DIR / "robustness_table.tex").write_text("\n".join(lines), encoding="utf-8")


def plot_results(rows: list[dict[str, object]]) -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.2), dpi=220)
    for ax, experiment, title in [
        (axes[0], "cbf_gain_sensitivity", "CBF gain sensitivity"),
        (axes[1], "actuation_stress", "Actuation-limit stress"),
    ]:
        subset = [row for row in rows if row["experiment"] == experiment]
        labels = [str(row["case"]) for row in subset]
        x = np.arange(len(labels))
        min_h = [float(row["min_h"]) for row in subset]
        mean_error = [float(row["mean_error"]) for row in subset]
        ax.bar(x - 0.18, min_h, width=0.36, label=r"$\min h$")
        ax.bar(x + 0.18, mean_error, width=0.36, label="mean error")
        ax.axhline(0.0, color="k", linestyle="--", linewidth=0.9)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=25, ha="right")
        ax.set_title(title, loc="left", fontsize=8.5)
        ax.grid(True, axis="y", alpha=0.3)
    axes[0].set_ylabel("metric value")
    axes[0].legend(fontsize=7)
    fig.tight_layout()
    save_publication_figure(fig, FIG_DIR / "robustness_checks")
    plt.close(fig)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    FIG_DIR.mkdir(parents=True, exist_ok=True)

    rows = run_sensitivity() + run_actuation_stress()
    fieldnames = [
        "experiment",
        "case",
        "min_h",
        "mean_error",
        "max_error",
        "qp_success_rate",
        "active_rate",
        "max_correction",
        "max_thrust",
        "thrust_saturation_ratio",
    ]
    write_csv(OUT_DIR / "robustness_summary.csv", rows, fieldnames)
    write_latex_table(rows)
    plot_results(rows)
    print((OUT_DIR / "robustness_summary.csv").read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
