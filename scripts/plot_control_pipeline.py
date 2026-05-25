from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "output" / "paper_figures" / "control_pipeline.png"


def add_box(ax, xy: tuple[float, float], width: float, height: float, title: str, body: str, fc: str) -> None:
    box = FancyBboxPatch(
        xy,
        width,
        height,
        boxstyle="round,pad=0.012,rounding_size=0.018",
        linewidth=1.2,
        edgecolor="#2f3a44",
        facecolor=fc,
    )
    ax.add_patch(box)
    x, y = xy
    ax.text(x + width / 2, y + height * 0.64, title, ha="center", va="center", fontsize=9.5, weight="bold")
    ax.text(x + width / 2, y + height * 0.34, body, ha="center", va="center", fontsize=8.2)


def arrow(ax, start: tuple[float, float], end: tuple[float, float], label: str = "") -> None:
    patch = FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=13, linewidth=1.1, color="#2f3a44")
    ax.add_patch(patch)
    if label:
        ax.text((start[0] + end[0]) / 2, (start[1] + end[1]) / 2 + 0.035, label, ha="center", fontsize=7.7)


def main() -> None:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(12, 4.6), dpi=220)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    y = 0.52
    w = 0.145
    h = 0.22
    xs = [0.035, 0.205, 0.375, 0.545, 0.715, 0.865]

    add_box(ax, (xs[0], y), w, h, "Reference and state", "$p_d,v_d,a_d$\\n$p,v,R,\\Omega$", "#f3f6f8")
    add_box(ax, (xs[1], y), w, h, "Residual DOB", "$\\hat d_p$ from\\nacceleration residual", "#edf7f3")
    add_box(ax, (xs[2], y), w, h, "Robust nominal\\ntracking", "$a_{nom}$ from errors,\\n$\\hat d_p$, robust term", "#eef4ff")
    add_box(ax, (xs[3], y), w, h, "High-order\\nCBF-QP", "$a_c$ satisfying\\nobstacle constraints", "#fff4e8")
    add_box(ax, (xs[4], y), w, h, "Thrust-attitude\\nconversion", "$T,R_d$ for\\n6-DOF quadrotor", "#f3efff")
    add_box(ax, (xs[5], y), 0.105, h, "Vehicle", "6-DOF model\\nor PX4 stack", "#f7f1f1")

    for i in range(5):
        start = (xs[i] + (w if i < 5 else 0.105), y + h / 2)
        end = (xs[i + 1], y + h / 2)
        arrow(ax, start, end)

    arrow(ax, (xs[5] + 0.052, y), (xs[0] + 0.07, 0.25), "measured state")
    arrow(ax, (xs[0] + 0.07, 0.25), (xs[0] + 0.07, y), "")

    ax.text(
        0.5,
        0.88,
        "Disturbance-observer-assisted CBF safety filtering pipeline",
        ha="center",
        va="center",
        fontsize=12,
        weight="bold",
    )
    ax.text(
        0.5,
        0.12,
        "The nominal controller improves tracking, the CBF-QP enforces safety under stated assumptions, "
        "and the vehicle layer tests execution through 6-DOF dynamics or PX4/Gazebo.",
        ha="center",
        va="center",
        fontsize=8.2,
    )

    fig.tight_layout(pad=0.2)
    fig.savefig(OUT)
    print(OUT)


if __name__ == "__main__":
    main()
