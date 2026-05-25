from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "output" / "paper_figures"
OUT_STEM = OUT_DIR / "control_pipeline"


def add_box(
    ax: plt.Axes,
    x: float,
    y: float,
    w: float,
    h: float,
    title: str,
    body: str,
    face: str,
    edge: str = "#26323a",
    title_size: float = 8.5,
    body_size: float = 7.2,
) -> None:
    patch = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.012,rounding_size=0.012",
        linewidth=1.05,
        edgecolor=edge,
        facecolor=face,
    )
    ax.add_patch(patch)
    ax.text(
        x + w / 2,
        y + h * 0.68,
        title,
        ha="center",
        va="center",
        fontsize=title_size,
        fontweight="bold",
        linespacing=1.12,
    )
    ax.text(
        x + w / 2,
        y + h * 0.34,
        body,
        ha="center",
        va="center",
        fontsize=body_size,
        linespacing=1.12,
    )


def add_arrow(
    ax: plt.Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    label: str = "",
    label_offset: tuple[float, float] = (0.0, 0.026),
    color: str = "#26323a",
    lw: float = 1.05,
    style: str = "-|>",
    dashed: bool = False,
) -> None:
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle=style,
        mutation_scale=11,
        linewidth=lw,
        color=color,
        linestyle="--" if dashed else "-",
        shrinkA=2,
        shrinkB=2,
    )
    ax.add_patch(arrow)
    if label:
        ax.text(
            (start[0] + end[0]) / 2 + label_offset[0],
            (start[1] + end[1]) / 2 + label_offset[1],
            label,
            ha="center",
            va="center",
            fontsize=6.8,
            color=color,
        )


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans", "sans-serif"],
            "svg.fonttype": "none",
            "pdf.fonttype": 42,
            "axes.linewidth": 0.8,
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )

    fig, ax = plt.subplots(figsize=(7.7, 4.45), dpi=300)
    ax.set_xlim(0, 1.04)
    ax.set_ylim(0, 1)
    ax.axis("off")

    # Main outer-loop chain.
    y_main = 0.57
    h_main = 0.18
    x0 = 0.045
    gap = 0.022
    widths = [0.125, 0.125, 0.145, 0.16, 0.135, 0.125]
    xs = [x0]
    for i in range(1, len(widths)):
        xs.append(xs[-1] + widths[i - 1] + gap)

    add_box(
        ax,
        xs[0],
        y_main,
        widths[0],
        h_main,
        "Reference\nand state",
        "$p_d,v_d,a_d$\n$p,v,R,\\Omega$",
        "#f2f5f7",
    )
    add_box(
        ax,
        xs[1],
        y_main,
        widths[1],
        h_main,
        "Tracking\nerrors",
        "$e_p,e_v$\nouter-loop state",
        "#edf4fb",
    )
    add_box(
        ax,
        xs[2],
        y_main,
        widths[2],
        h_main,
        "DOB-assisted\nrobust control",
        "$\\hat d_p$ and robust term\nproduce $a_{nom}$",
        "#edf7f2",
    )
    add_box(
        ax,
        xs[3],
        y_main,
        widths[3],
        h_main,
        "High-order\nCBF-QP filter",
        "enforces obstacle\nand input constraints",
        "#fff4e7",
    )
    add_box(
        ax,
        xs[4],
        y_main,
        widths[4],
        h_main,
        "Thrust-attitude\nrealization",
        "$a_c \\rightarrow T,R_d$\nSO(3) attitude loop",
        "#f1edfb",
    )
    add_box(
        ax,
        xs[5],
        y_main,
        widths[5],
        h_main,
        "Quadrotor\nplant",
        "6-DOF simulation\nor PX4/Gazebo",
        "#f8eeee",
    )

    for i in range(len(widths) - 1):
        add_arrow(
            ax,
            (xs[i] + widths[i], y_main + h_main / 2),
            (xs[i + 1], y_main + h_main / 2),
        )

    # Constraint and observer side information.
    add_box(
        ax,
        0.43,
        0.28,
        0.20,
        0.13,
        "Safety set",
        "$h(p) \\geq 0$, $\\dot h$, $\\ddot h$\nacceleration bounds",
        "#f7f2e8",
        title_size=8.0,
        body_size=6.8,
    )
    add_box(
        ax,
        0.08,
        0.28,
        0.155,
        0.13,
        "Residual\nobserver input",
        "acceleration mismatch\nand measured state",
        "#edf7f2",
        title_size=8.0,
        body_size=6.8,
    )

    add_arrow(
        ax,
        (0.47, 0.41),
        (xs[3] + widths[3] * 0.48, y_main),
        label="CBF constraints",
        label_offset=(0.03, 0.015),
    )
    add_arrow(
        ax,
        (0.157, 0.41),
        (xs[2] + widths[2] * 0.35, y_main),
        label="$\\hat d_p$",
        label_offset=(-0.018, 0.008),
    )

    # Feedback path: the plant output feeds both tracking-error computation and the observer.
    plant_bottom = (xs[5] + widths[5] * 0.55, y_main)
    feedback_y = 0.19
    feedback_right = (xs[5] + widths[5] * 0.55, feedback_y)
    feedback_error = (xs[1] + widths[1] * 0.52, feedback_y)
    feedback_observer = (0.255, feedback_y)
    ax.plot(
        [plant_bottom[0], feedback_right[0], feedback_error[0]],
        [plant_bottom[1], feedback_y, feedback_y],
        color="#26323a",
        linewidth=1.0,
    )
    ax.text(
        (feedback_right[0] + feedback_error[0]) / 2,
        feedback_y + 0.03,
        "measured state",
        ha="center",
        va="center",
        fontsize=6.8,
        color="#26323a",
    )
    add_arrow(
        ax,
        feedback_error,
        (xs[1] + widths[1] * 0.52, y_main),
        lw=1.0,
    )
    add_arrow(
        ax,
        feedback_observer,
        (0.235, 0.345),
        lw=0.95,
    )

    # Claim boundary.
    ax.text(
        0.5,
        0.91,
        "Disturbance-observer-assisted CBF safety-filter architecture",
        ha="center",
        va="center",
        fontsize=10.6,
        fontweight="bold",
    )
    ax.text(
        0.5,
        0.06,
        "The theorem is stated for the acceleration-level outer loop; 6-DOF and PX4/Gazebo results validate execution layers.",
        ha="center",
        va="center",
        fontsize=7.1,
    )

    fig.tight_layout(pad=0.15)
    for suffix in (".png", ".pdf", ".svg"):
        fig.savefig(OUT_STEM.with_suffix(suffix), bbox_inches="tight")
    print(OUT_STEM.with_suffix(".png"))


if __name__ == "__main__":
    main()
