# Copyright (c) 2025 IBM
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import List, Dict

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


def configure_plot_style() -> None:
    """Set a photogenic Matplotlib style for the DNN results plot."""
    plt.style.use("default")
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["DejaVu Serif", "CMU Serif", "Times New Roman"],
            "font.size": 12,
            "axes.prop_cycle": mpl.cycler(
                color=[plt.cm.viridis(i) for i in np.linspace(0.1, 1, 6)]
            ),
        }
    )


def read_results(csv_path: Path) -> List[Dict[str, str]]:
    with csv_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return list(reader)


def plot_results(rows: List[Dict[str, str]], output_path: Path) -> None:
    grouped: Dict[str, List[Dict[str, str]]] = {}
    for row in rows:
        grouped.setdefault(row["model"], []).append(row)

    positions: List[float] = []
    labels: List[str] = []
    model_centers: List[float] = []
    separators: List[float] = []
    current_pos = 0.0
    gap = 0.6  # visual separation between models
    models = list(grouped.items())
    for idx, (model, items) in enumerate(models):
        start_pos = current_pos
        for item in items:
            positions.append(current_pos)
            labels.append(f"{item['transformation']} ({item['matches']})")
            current_pos += 1.0
        model_centers.append((start_pos + current_pos - 1) / 2.0)
        if idx < len(models) - 1:
            separators.append(current_pos + gap / 2.0 - gap)
        current_pos += gap

    datasets = [
        ("GHL (this work)", [float(row["GHL"]) for row in rows]),
        ("PyTorch FX", [float(row["PyTorch FX"]) for row in rows]),
        ("Apache TVM", [float(row["Apache TVM"]) for row in rows]),
    ]

    total_width = 0.82
    bar_width = total_width / len(datasets)
    fig, ax = plt.subplots(figsize=(12, 4))
    edge_color = "#0f172a"

    for idx, (label, values) in enumerate(datasets):
        offset = -total_width / 2 + idx * bar_width + bar_width / 2
        bar_positions = [p + offset for p in positions]
        ax.bar(
            bar_positions,
            values,
            width=bar_width,
            label=label,
            edgecolor=edge_color,
            linewidth=0.8,
            alpha=0.9,
        )

    ax.set_xticks(positions)
    ax.set_xticklabels(labels, rotation=20, ha="right")
    top_ax = ax.secondary_xaxis("top")
    top_ax.set_xticks(model_centers)
    top_ax.set_xticklabels(list(grouped.keys()), fontsize=11)
    top_ax.tick_params(axis="x", pad=6)

    for sep in separators:
        ax.axvline(sep, color=edge_color, linestyle="--", linewidth=1.2, alpha=0.85)

    ax.set_ylabel("Mean Time (s)")
    ax.set_title("DNN Transformation Latency")
    ax.set_yscale("log")
    ax.grid(True, linestyle="--", alpha=0.35, which="both")
    ax.set_axisbelow(True)
    ax.legend(loc="lower right", frameon=True, framealpha=0.95, ncol=3)
    for spine in ax.spines.values():
        spine.set_visible(True)
        spine.set_linewidth(1.0)
        spine.set_edgecolor(edge_color)
    for spine in top_ax.spines.values():
        spine.set_visible(True)
        spine.set_linewidth(1.0)
        spine.set_edgecolor(edge_color)
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    print(f"Saved plot to {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot DNN transformation timings from results.csv"
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path(__file__).parent / "results" / "results.csv",
        help="Path to results.csv",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).parent / "results" / "dnn_results.pdf",
        help="Output plot path (image)",
    )
    args = parser.parse_args()

    configure_plot_style()
    rows = read_results(args.csv)
    if not rows:
        raise ValueError(f"No rows found in {args.csv}")
    plot_results(rows, args.output)


if __name__ == "__main__":
    main()
