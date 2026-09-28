"""Render a report-ready PNG corresponding to docs/system_architecture.mmd."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parent.parent.parent
OUTPUT = ROOT / "docs" / "system_architecture.png"


def box(ax, x, y, width, height, text, color, dashed=False):
    patch = FancyBboxPatch(
        (x, y), width, height,
        boxstyle="round,pad=0.018,rounding_size=0.02",
        linewidth=1.6,
        edgecolor="#263238",
        facecolor=color,
        linestyle="--" if dashed else "-",
    )
    ax.add_patch(patch)
    ax.text(x + width / 2, y + height / 2, text, ha="center", va="center",
            fontsize=9, color="#172026", wrap=True)
    return patch


def arrow(ax, start, end, label="", dashed=False, color="#455a64"):
    patch = FancyArrowPatch(
        start, end, arrowstyle="-|>", mutation_scale=12,
        linewidth=1.5, color=color,
        linestyle="--" if dashed else "-",
        connectionstyle="arc3,rad=0.0",
    )
    ax.add_patch(patch)
    if label:
        ax.text((start[0] + end[0]) / 2, (start[1] + end[1]) / 2 + 0.018,
                label, ha="center", va="bottom", fontsize=7.5, color=color,
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.9, pad=1))


fig, ax = plt.subplots(figsize=(16, 9), dpi=180)
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.axis("off")
fig.patch.set_facecolor("white")
ax.set_title("Multi-QCar Autonomous Driving System Architecture",
             fontsize=18, fontweight="bold", pad=18)

# Group backgrounds
for x, title, color in [(0.03, "LOCAL COMPUTER — 192.168.0.153", "#e3f2fd"),
                        (0.37, "QCAR .151", "#e8f5e9"),
                        (0.70, "QCAR .152", "#fff3e0")]:
    ax.add_patch(FancyBboxPatch((x, 0.10), 0.27, 0.79,
                               boxstyle="round,pad=0.015",
                               facecolor=color, edgecolor="#607d8b",
                               linewidth=2))
    ax.text(x + 0.135, 0.855, title, ha="center", va="center",
            fontsize=11, fontweight="bold")

# Local computer
box(ax, 0.07, 0.70, 0.19, 0.09, "Launcher\nSSH/SCP deployment", "#bbdefb")
box(ax, 0.07, 0.48, 0.19, 0.10, "CZ1 Intersection Coordinator\nUDP port 18750", "#90caf9")
box(ax, 0.07, 0.23, 0.19, 0.10, "Observer, CSV Logs,\nand Experiment Evidence", "#bbdefb")
arrow(ax, (0.165, 0.70), (0.165, 0.58))

# QCar .151
box(ax, 0.41, 0.70, 0.19, 0.08, "GPS/LiDAR + EKF", "#c8e6c9")
box(ax, 0.41, 0.58, 0.19, 0.08, "Route + Path Tracking", "#c8e6c9")
box(ax, 0.41, 0.45, 0.19, 0.09, "Vehicle Control\nSpeed + Steering", "#81c784")
box(ax, 0.41, 0.32, 0.09, 0.08, "YOLO\nSafety", "#c8e6c9")
box(ax, 0.51, 0.32, 0.09, 0.08, "Front CSI\nWarning", "#fff9c4")
box(ax, 0.41, 0.18, 0.19, 0.08, "Steering + Throttle", "#a5d6a7")
arrow(ax, (0.505, 0.70), (0.505, 0.54))
arrow(ax, (0.505, 0.58), (0.505, 0.54))
arrow(ax, (0.455, 0.32), (0.475, 0.45))
arrow(ax, (0.555, 0.32), (0.535, 0.45), "measurement", True, "#ef6c00")
arrow(ax, (0.505, 0.45), (0.505, 0.26))

# QCar .152
box(ax, 0.74, 0.70, 0.19, 0.08, "GPS/LiDAR + EKF", "#ffe0b2")
box(ax, 0.74, 0.58, 0.19, 0.08, "Route + Path Tracking", "#ffe0b2")
box(ax, 0.74, 0.45, 0.19, 0.09, "Vehicle Control\nSpeed + Steering", "#ffb74d")
box(ax, 0.74, 0.32, 0.19, 0.08, "YOLO Obstacle Safety", "#ffe0b2")
box(ax, 0.74, 0.18, 0.19, 0.08, "Steering + Throttle", "#ffcc80")
arrow(ax, (0.835, 0.70), (0.835, 0.54))
arrow(ax, (0.835, 0.58), (0.835, 0.54))
arrow(ax, (0.835, 0.32), (0.835, 0.45))
arrow(ax, (0.835, 0.45), (0.835, 0.26))

# Cross-system communication
arrow(ax, (0.26, 0.745), (0.41, 0.745), "deploy/start")
arrow(ax, (0.26, 0.72), (0.74, 0.72), "deploy/start")
arrow(ax, (0.41, 0.49), (0.26, 0.53), "state / grant")
arrow(ax, (0.74, 0.49), (0.26, 0.50), "state / grant")
arrow(ax, (0.41, 0.36), (0.26, 0.28), "diagnostics")

ax.text(0.505, 0.125,
        "Candidate 5: earlier slowdown/latched stop (physical validation pending)",
        ha="center", va="center", fontsize=9, color="#bf360c",
        bbox=dict(boxstyle="round,pad=0.35", facecolor="#fff3e0",
                  edgecolor="#ef6c00", linestyle="--"))

fig.savefig(OUTPUT, bbox_inches="tight", facecolor="white")
print(OUTPUT)
