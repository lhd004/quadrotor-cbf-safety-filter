from __future__ import annotations

import argparse
import csv
import glob
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle


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
        "figure.facecolor": "white",
        "axes.facecolor": "white",
    }
)

SCENARIO_LABELS = {
    "single_obstacle": "Single obstacle",
    "large_obstacle": "Large obstacle",
    "multi_obstacle": "Multiple obstacles",
    "narrow_passage": "Narrow passage",
    "wind_bias": "Wind-bias stress",
}

CONTROLLER_LABELS = {
    "baseline_no_cbf": "Baseline",
    "cbf_velocity_filter": "CBF filter",
}

CONTROLLER_STYLES = {
    "baseline_no_cbf": {"color": "#555555", "linestyle": "--", "linewidth": 1.8},
    "cbf_velocity_filter": {"color": "#2A6FBB", "linestyle": "-", "linewidth": 2.0},
}


def save_publication_figure(fig: plt.Figure, stem: Path) -> None:
    fig.savefig(stem.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(stem.with_suffix(".pdf"), bbox_inches="tight")
    fig.savefig(stem.with_suffix(".svg"), bbox_inches="tight")
    fig.savefig(stem.with_suffix(".tiff"), dpi=600, bbox_inches="tight")


def scenario_label(name: str) -> str:
    return SCENARIO_LABELS.get(name, name.replace("_", " ").title())


def controller_label(name: str) -> str:
    return CONTROLLER_LABELS.get(name, name.replace("_", " "))


@dataclass
class CaseData:
    name: str
    scenario: str
    controller: str
    obstacles: list[tuple[float, float, float]]
    path: Path
    t: list[float]
    x: list[float]
    y: list[float]
    z: list[float]
    h: list[float]
    goal_distance: list[float]
    correction: list[float]
    wind_vx: list[float]
    wind_vy: list[float]


def parse_obstacles(raw: str) -> list[tuple[float, float, float]]:
    obstacles: list[tuple[float, float, float]] = []
    for item in raw.split(";"):
        item = item.strip()
        if not item:
            continue
        values = [float(part) for part in item.split(",")]
        if len(values) == 3:
            obstacles.append((values[0], values[1], values[2]))
    return obstacles


def infer_labels(path: Path, first_row: dict[str, str]) -> tuple[str, str]:
    if first_row.get("scenario") and first_row.get("controller"):
        return first_row["scenario"], first_row["controller"]
    name = path.stem
    for suffix in ("_baseline_no_cbf", "_cbf_velocity_filter"):
        if name.endswith(suffix):
            return name[: -len(suffix)] or "single_obstacle", suffix[1:]
    if name == "baseline_no_cbf":
        return "single_obstacle", "baseline_no_cbf"
    if name == "cbf_velocity_filter":
        return "single_obstacle", "cbf_velocity_filter"
    return "single_obstacle", name


def load_case(path: Path) -> CaseData:
    rows = list(csv.DictReader(path.open(newline="", encoding="utf-8")))
    if not rows:
        raise RuntimeError(f"No rows in {path}")
    first = rows[0]
    required = {"t", "x", "y", "z", "h", "goal_distance", "correction"}
    missing = required.difference(first)
    if missing:
        raise RuntimeError(f"{path} is not a trajectory log; missing columns: {sorted(missing)}")
    scenario, controller = infer_labels(path, first)
    obstacles = parse_obstacles(first.get("obstacles", ""))
    if not obstacles:
        obstacles = [(3.0, 0.0, 1.15)]
    return CaseData(
        name=path.stem,
        scenario=scenario,
        controller=controller,
        obstacles=obstacles,
        path=path,
        t=[float(r["t"]) for r in rows],
        x=[float(r["x"]) for r in rows],
        y=[float(r["y"]) for r in rows],
        z=[float(r["z"]) for r in rows],
        h=[float(r["h"]) for r in rows],
        goal_distance=[float(r["goal_distance"]) for r in rows],
        correction=[float(r["correction"]) for r in rows],
        wind_vx=[float(r.get("wind_vx", 0.0) or 0.0) for r in rows],
        wind_vy=[float(r.get("wind_vy", 0.0) or 0.0) for r in rows],
    )


def case_sort_key(case: CaseData) -> tuple[str, int, str]:
    order = {"baseline_no_cbf": 0, "cbf_velocity_filter": 1}
    return case.scenario, order.get(case.controller, 99), case.controller


def prefer_explicit_scenario_logs(cases: list[CaseData]) -> list[CaseData]:
    explicit_keys = {
        (case.scenario, case.controller)
        for case in cases
        if case.path.stem.startswith(f"{case.scenario}_")
    }
    filtered: list[CaseData] = []
    for case in cases:
        key = (case.scenario, case.controller)
        if key in explicit_keys and not case.path.stem.startswith(f"{case.scenario}_"):
            continue
        filtered.append(case)
    return filtered


def write_summary(cases: list[CaseData], output_dir: Path) -> None:
    summary_path = output_dir / "summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "scenario",
                "controller",
                "case",
                "rows",
                "min_h",
                "final_x",
                "final_y",
                "final_z",
                "final_goal_distance",
                "max_correction",
                "mean_wind_vx",
                "mean_wind_vy",
            ],
        )
        writer.writeheader()
        for case in sorted(cases, key=case_sort_key):
            writer.writerow(
                {
                    "scenario": case.scenario,
                    "controller": case.controller,
                    "case": case.name,
                    "rows": len(case.t),
                    "min_h": f"{min(case.h):.6f}",
                    "final_x": f"{case.x[-1]:.6f}",
                    "final_y": f"{case.y[-1]:.6f}",
                    "final_z": f"{case.z[-1]:.6f}",
                    "final_goal_distance": f"{case.goal_distance[-1]:.6f}",
                    "max_correction": f"{max(case.correction):.6f}",
                    "mean_wind_vx": f"{sum(case.wind_vx) / len(case.wind_vx):.6f}",
                    "mean_wind_vy": f"{sum(case.wind_vy) / len(case.wind_vy):.6f}",
                }
            )


def write_latex_table(cases: list[CaseData], output_dir: Path) -> None:
    lines = [
        r"\begin{table}[t]",
        r"\caption{PX4 SITL + Gazebo multi-scenario offboard validation. Positive $\min h$ indicates that the logged trajectory remains outside the specified obstacle set.}",
        r"\label{tab:px4_multiscenario}",
        r"\centering",
        r"\begin{tabular}{llrr}",
        r"\toprule",
        r"Scenario & Controller & $\min h \uparrow$ & Final goal dist. \\",
        r"\midrule",
    ]
    for case in sorted(cases, key=case_sort_key):
        scenario = case.scenario.replace("_", r"\_")
        controller = case.controller.replace("_", r"\_")
        lines.append(
            f"{scenario} & {controller} & {min(case.h):.3f} & {case.goal_distance[-1]:.3f} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table}", ""])
    (output_dir / "summary_table.tex").write_text("\n".join(lines), encoding="utf-8")


def plot_pair(cases: list[CaseData], output_dir: Path, filename: str, title: str) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), dpi=180)
    for case in sorted(cases, key=case_sort_key):
        style = CONTROLLER_STYLES.get(case.controller, {})
        axes[0].plot(case.x, case.y, label=controller_label(case.controller), **style)
        axes[1].plot(case.t, case.h, label=controller_label(case.controller), **style)
    for ox, oy, radius in cases[0].obstacles:
        circle = Circle((ox, oy), radius, fill=True, facecolor="#F3C7C7", edgecolor="#A83232", alpha=0.45, linewidth=1.0)
        axes[0].add_patch(circle)
        axes[0].scatter([ox], [oy], color="#A83232", s=12, zorder=3)
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_xlabel("x NED [m]")
    axes[0].set_ylabel("y NED [m]")
    axes[0].set_title(f"{title}: trajectory")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    axes[1].axhline(0.0, color="#A83232", linestyle=":", linewidth=1.2)
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("CBF barrier h")
    axes[1].set_title(f"{title}: safety function")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / filename)
    plt.close(fig)


def plot_multi_scenario_trajectories(cases: list[CaseData], output_dir: Path) -> None:
    grouped: dict[str, list[CaseData]] = defaultdict(list)
    for case in cases:
        grouped[case.scenario].append(case)
    preferred_order = ["single_obstacle", "large_obstacle", "multi_obstacle", "narrow_passage", "wind_bias"]
    scenarios = [name for name in preferred_order if name in grouped]
    scenarios.extend(sorted(name for name in grouped if name not in scenarios))

    cols = 2
    rows = max(1, math.ceil(len(scenarios) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(7.2, 3.2 * rows), dpi=220)
    flat_axes = axes.flatten() if hasattr(axes, "flatten") else [axes]
    panel_letters = list("abcdefghijklmnopqrstuvwxyz")

    for idx, (ax, scenario) in enumerate(zip(flat_axes, scenarios)):
        scenario_cases = sorted(grouped[scenario], key=case_sort_key)
        all_x: list[float] = []
        all_y: list[float] = []
        for case in scenario_cases:
            all_x.extend(case.x)
            all_y.extend(case.y)
            style = CONTROLLER_STYLES.get(case.controller, {})
            ax.plot(case.x, case.y, label=controller_label(case.controller), **style)
            ax.scatter(case.x[0], case.y[0], s=18, marker="o", color=style.get("color", "black"), zorder=4)
            ax.scatter(case.x[-1], case.y[-1], s=28, marker="x", color=style.get("color", "black"), zorder=4)

        for ox, oy, radius in scenario_cases[0].obstacles:
            all_x.extend([ox - radius, ox + radius])
            all_y.extend([oy - radius, oy + radius])
            ax.add_patch(
                Circle(
                    (ox, oy),
                    radius,
                    facecolor="#F3C7C7",
                    edgecolor="#A83232",
                    linewidth=1.0,
                    alpha=0.55,
                    zorder=1,
                )
            )
            ax.scatter([ox], [oy], color="#A83232", s=10, zorder=5)

        pad = 0.35
        ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
        ax.set_ylim(min(all_y) - pad, max(all_y) + pad)
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(scenario_label(scenario), loc="left", pad=4, fontsize=8.5)
        ax.text(-0.13, 1.05, panel_letters[idx], transform=ax.transAxes, fontweight="bold", fontsize=9)
        ax.set_xlabel("x NED [m]")
        ax.set_ylabel("y NED [m]")
        ax.grid(True, color="#D7D7D7", linewidth=0.5, alpha=0.6)

    for ax in flat_axes[len(scenarios) :]:
        ax.axis("off")

    handles, labels = flat_axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.0), handlelength=2.8)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
    save_publication_figure(fig, output_dir / "multi_scenario_trajectories")
    plt.close(fig)


def plot_multi_scenario(cases: list[CaseData], output_dir: Path) -> None:
    grouped: dict[str, list[CaseData]] = defaultdict(list)
    for case in cases:
        grouped[case.scenario].append(case)
    preferred_order = ["single_obstacle", "large_obstacle", "multi_obstacle", "narrow_passage", "wind_bias"]
    scenarios = [name for name in preferred_order if name in grouped]
    scenarios.extend(sorted(name for name in grouped if name not in scenarios))
    cols = 2
    rows = max(1, math.ceil(len(scenarios) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(7.2, 2.9 * rows), dpi=220)
    flat_axes = axes.flatten() if hasattr(axes, "flatten") else [axes]
    panel_letters = list("abcdefghijklmnopqrstuvwxyz")
    for idx, (ax, scenario) in enumerate(zip(flat_axes, scenarios)):
        scenario_cases = sorted(grouped[scenario], key=case_sort_key)
        for case in scenario_cases:
            style = CONTROLLER_STYLES.get(case.controller, {})
            ax.plot(case.t, case.h, label=controller_label(case.controller), **style)
        ax.axhline(0.0, color="#A83232", linestyle=":", linewidth=1.2)
        ax.set_title(scenario_label(scenario), loc="left", pad=4, fontsize=8.5)
        ax.text(-0.13, 1.05, panel_letters[idx], transform=ax.transAxes, fontweight="bold", fontsize=9)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("barrier value h")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    for ax in flat_axes[len(scenarios) :]:
        ax.axis("off")
    fig.tight_layout()
    save_publication_figure(fig, output_dir / "multi_scenario_safety")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("output/px4_offboard_experiments"))
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    cases: list[CaseData] = []
    csv_paths: list[Path] = []
    for path in args.csv:
        matches = sorted(Path(match) for match in glob.glob(str(path)))
        csv_paths.extend(matches or [path])
    for path in csv_paths:
        try:
            cases.append(load_case(path))
        except RuntimeError as exc:
            print(f"Skipping {path}: {exc}")
    if not cases:
        raise RuntimeError("No trajectory logs were loaded.")
    cases = prefer_explicit_scenario_logs(cases)
    write_summary(cases, args.output_dir)
    write_latex_table(cases, args.output_dir)
    plot_multi_scenario(cases, args.output_dir)
    plot_multi_scenario_trajectories(cases, args.output_dir)

    grouped: dict[str, list[CaseData]] = defaultdict(list)
    for case in cases:
        grouped[case.scenario].append(case)
    for scenario, scenario_cases in grouped.items():
        plot_pair(
            scenario_cases,
            args.output_dir,
            f"{scenario}_comparison.png",
            scenario.replace("_", " "),
        )
    if "single_obstacle" in grouped:
        plot_pair(grouped["single_obstacle"], args.output_dir, "comparison.png", "single obstacle")

    for case in sorted(cases, key=case_sort_key):
        print(
            f"{case.scenario}/{case.controller}: rows={len(case.t)}, min_h={min(case.h):.6f}, "
            f"final=({case.x[-1]:.3f},{case.y[-1]:.3f},{case.z[-1]:.3f})"
        )


if __name__ == "__main__":
    main()
