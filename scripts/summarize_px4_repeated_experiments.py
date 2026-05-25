from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path


def read_case(path: Path) -> dict[str, object] | None:
    rows = list(csv.DictReader(path.open(newline="", encoding="utf-8")))
    if not rows:
        return None
    first = rows[0]
    required = {"scenario", "controller", "h", "goal_distance", "correction"}
    if not required.issubset(first):
        return None
    h_values = [float(row["h"]) for row in rows]
    goal_values = [float(row["goal_distance"]) for row in rows]
    corr_values = [float(row["correction"]) for row in rows]
    stem = path.stem
    repeat = "r00"
    if stem.startswith("r") and "_" in stem:
        repeat = stem.split("_", 1)[0]
    return {
        "repeat": repeat,
        "scenario": first["scenario"],
        "controller": first["controller"],
        "case": stem,
        "rows": len(rows),
        "min_h": min(h_values),
        "final_goal_distance": goal_values[-1],
        "max_correction": max(corr_values),
        "success": 1 if min(h_values) > 0.0 else 0,
        "mean_wind_vx": statistics.fmean(float(row.get("wind_vx", 0.0) or 0.0) for row in rows),
        "mean_wind_vy": statistics.fmean(float(row.get("wind_vy", 0.0) or 0.0) for row in rows),
    }


def mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else math.nan


def std(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) > 1 else 0.0


def format_float(value: float, digits: int = 6) -> str:
    if math.isnan(value):
        return "nan"
    return f"{value:.{digits}f}"


def write_summary(rows: list[dict[str, object]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    detail_path = output_dir / "repeated_runs_detail.csv"
    fieldnames = [
        "repeat",
        "scenario",
        "controller",
        "case",
        "rows",
        "min_h",
        "final_goal_distance",
        "max_correction",
        "success",
        "mean_wind_vx",
        "mean_wind_vy",
    ]
    with detail_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in sorted(rows, key=lambda r: (str(r["scenario"]), str(r["controller"]), str(r["repeat"]))):
            writer.writerow({key: row[key] for key in fieldnames})

    grouped: dict[tuple[str, str], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[(str(row["scenario"]), str(row["controller"]))].append(row)

    stats_path = output_dir / "repeated_runs_summary.csv"
    stats_fields = [
        "scenario",
        "controller",
        "n",
        "min_h_mean",
        "min_h_std",
        "min_h_min",
        "goal_distance_mean",
        "goal_distance_std",
        "success_rate",
        "max_correction_mean",
    ]
    stats_rows: list[dict[str, object]] = []
    for (scenario, controller), group in sorted(grouped.items()):
        min_h_values = [float(row["min_h"]) for row in group]
        goal_values = [float(row["final_goal_distance"]) for row in group]
        correction_values = [float(row["max_correction"]) for row in group]
        success_values = [float(row["success"]) for row in group]
        stats_rows.append(
            {
                "scenario": scenario,
                "controller": controller,
                "n": len(group),
                "min_h_mean": mean(min_h_values),
                "min_h_std": std(min_h_values),
                "min_h_min": min(min_h_values),
                "goal_distance_mean": mean(goal_values),
                "goal_distance_std": std(goal_values),
                "success_rate": mean(success_values),
                "max_correction_mean": mean(correction_values),
            }
        )
    with stats_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=stats_fields)
        writer.writeheader()
        for row in stats_rows:
            writer.writerow(
                {
                    key: format_float(row[key]) if isinstance(row[key], float) else row[key]
                    for key in stats_fields
                }
            )

    lines = [
        r"\begin{table}[t]",
        r"\caption{Repeatability of PX4 SITL + Gazebo offboard validation over repeated deterministic start-up runs. Positive $\min h$ indicates that the logged trajectory remains outside the specified obstacle set.}",
        r"\label{tab:px4_repeatability}",
        r"\centering",
        r"\scriptsize",
        r"\resizebox{\linewidth}{!}{%",
        r"\begin{tabular}{llrrrrr}",
        r"\toprule",
        r"Scenario & Controller & $n$ & mean $\min h$ & std $\min h$ & Success & Mean final dist. \\",
        r"\midrule",
    ]
    controller_order = {"baseline_no_cbf": 0, "cbf_velocity_filter": 1}
    stats_rows.sort(key=lambda r: (str(r["scenario"]), controller_order.get(str(r["controller"]), 99)))
    for row in stats_rows:
        scenario = str(row["scenario"]).replace("_", r"\_")
        controller = str(row["controller"]).replace("_", r"\_")
        lines.append(
            f"{scenario} & {controller} & {row['n']} & "
            f"{float(row['min_h_mean']):.3f} & {float(row['min_h_std']):.3f} & "
            f"{100.0 * float(row['success_rate']):.0f}\\% & {float(row['goal_distance_mean']):.3f} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{tabular}%", r"}", r"\end{table}", ""])
    (output_dir / "repeatability_table.tex").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("output/px4_offboard_repeated"))
    args = parser.parse_args()

    rows = []
    for path in args.csv:
        for csv_path in sorted(path.parent.glob(path.name)):
            row = read_case(csv_path)
            if row is not None:
                rows.append(row)
    if not rows:
        raise RuntimeError("No repeated PX4 trajectory logs found.")
    write_summary(rows, args.output_dir)
    print(f"Wrote repeatability summary for {len(rows)} logs to {args.output_dir}")


if __name__ == "__main__":
    main()
