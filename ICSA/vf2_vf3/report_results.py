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
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CANDIDATE_SORT_COLUMNS = [
    "vf2_matches",
    "target_vertices",
    "target_edges",
    "target_density",
    "pattern_vertices",
    "pattern_edges",
    "pattern_density",
]


def factorize_series(series: pd.Series) -> pd.Series:
    """Convert a Series to a numeric representation for correlation.

    Non-numeric columns are factorized; numeric columns are returned as float.
    """

    if pd.api.types.is_numeric_dtype(series):
        return series.astype(float)
    codes, _ = pd.factorize(series, sort=False)
    return pd.Series(codes, index=series.index, dtype=float)


def correlation_sorted_columns(df: pd.DataFrame, target: str) -> list[str]:
    """Order candidate columns by absolute correlation to `target` (desc)."""

    scored = []
    target_series = factorize_series(df[target])
    for col in CANDIDATE_SORT_COLUMNS:
        if col not in df.columns:
            continue
        col_series = factorize_series(df[col])
        corr = target_series.corr(col_series)
        if pd.isna(corr):
            continue
        scored.append((abs(corr), col))
    scored.sort(key=lambda x: x[0], reverse=True)

    print(f"Correlations between parameters and {target}")
    for corr, key in scored:
        print(f"{key}: {corr}")
    return [col for _, col in scored]


def add_density_columns(df: pd.DataFrame) -> pd.DataFrame:
    def density(v_col: str, e_col: str) -> pd.Series:
        verts = df[v_col].astype(float)
        edges = df[e_col].astype(float)
        denom = verts * (verts - 1)
        dens = (2.0 * edges) / denom
        dens[denom == 0] = pd.NA
        return dens

    if {"target_vertices", "target_edges"}.issubset(df.columns):
        df["target_density"] = density("target_vertices", "target_edges")
    if {"pattern_vertices", "pattern_edges"}.issubset(df.columns):
        df["pattern_density"] = density("pattern_vertices", "pattern_edges")
    return df


def sort_dataframe(df: pd.DataFrame) -> pd.DataFrame:
    correlation_sorted_columns(df, target="vf3_mean")
    sorted_cols = correlation_sorted_columns(df, target="vf2_mean")
    if not sorted_cols:
        return df.sort_values(by=["vf2_matches"], kind="mergesort").reset_index(
            drop=True
        )
    return df.sort_values(
        by=["vf2_matches", *sorted_cols], kind="mergesort"
    ).reset_index(drop=True)


def find_last_index_leq(series: pd.Series, threshold: float) -> int | None:
    matches = series[series <= threshold]
    if matches.empty:
        return None
    return matches.index[-1]


def configure_plot_style() -> None:
    """Set a photogenic Matplotlib style for the results plot."""
    preferred_style = "default"
    plt.style.use(preferred_style)
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["DejaVu Serif", "CMU Serif", "Times New Roman"],
            "font.size": 18,
        }
    )


def plot_results(df: pd.DataFrame, output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 6))
    x = range(len(df))
    colors = [plt.cm.viridis(i) for i in np.linspace(0.1, 1, 6)]
    ax.scatter(
        x, df["vf2_mean"], marker="s", color=colors[0], label="VF2", alpha=0.9, s=70
    )
    ax.scatter(
        x, df["vf3_mean"], marker="^", color=colors[3], label="VF3", alpha=0.9, s=70
    )

    default_thresholds = [10, 100, 768, 1_000, 1_600, 1_000_000, 17_694_720]
    for thresh in default_thresholds:
        idx = find_last_index_leq(df["vf2_matches"], thresh)
        if idx is None:
            continue
        thresh_label = f"{thresh:.1E}" if thresh >= 100_000 else f"{thresh}"
        ax.axvline(x=idx + 3.0, linestyle="--", color="black", linewidth=0.8, alpha=0.8)
        ax.annotate(
            f"<= {thresh_label} matches",
            xy=(idx + 0.5, ax.get_ylim()[1]),
            xytext=(6, -100),
            textcoords="offset points",
            ha="left",
            va="center",
            fontsize=15,
            color="black",
            rotation=90,
        )

    ax.set_xlabel("Run Index (sorted by correlation between parameter and latency)")
    ax.set_ylabel("Time (s)")
    ax.set_title("VF2 vs VF3 Latencies")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.set_axisbelow(True)
    ax.legend(frameon=True, framealpha=0.95, markerscale=1.6)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    fig.tight_layout()
    fig.savefig(output_path)
    print(f"Saved plot to {output_path}")


def compute_stats(perf_df: pd.DataFrame, aborted_df: pd.DataFrame) -> None:
    perf_vf2_success = perf_df[perf_df["vf2_mean"] != -1]
    perf_vf3_success = perf_df[perf_df["vf3_mean"] != -1]

    aborted_vf2_success = aborted_df[aborted_df["vf2_mean"] != -1]
    aborted_vf3_success = aborted_df[aborted_df["vf3_mean"] != -1]

    vf2_success_total = len(perf_vf2_success) + len(aborted_vf2_success)
    vf3_success_total = len(perf_vf3_success) + len(aborted_vf3_success)

    vf2_fail_total = len(aborted_df[aborted_df["vf2_mean"] == -1])
    vf3_fail_total = len(aborted_df[aborted_df["vf3_mean"] == -1])

    vf2_success_vf3_aborted = len(
        aborted_df[(aborted_df["vf2_mean"] != -1) & (aborted_df["vf3_mean"] == -1)]
    )
    vf3_success_vf2_aborted = len(
        aborted_df[(aborted_df["vf3_mean"] != -1) & (aborted_df["vf2_mean"] == -1)]
    )

    combined_success = pd.concat(
        [
            perf_df[(perf_df["vf2_mean"] != -1) & (perf_df["vf3_mean"] != -1)],
            aborted_df[(aborted_df["vf2_mean"] != -1) & (aborted_df["vf3_mean"] != -1)],
        ],
        ignore_index=True,
    )
    vf3_faster = len(
        combined_success[combined_success["vf3_mean"] < combined_success["vf2_mean"]]
    )
    vf2_faster = len(
        combined_success[combined_success["vf2_mean"] < combined_success["vf3_mean"]]
    )

    all_vf2_success = pd.concat(
        [perf_vf2_success, aborted_vf2_success], ignore_index=True
    )
    all_vf3_success = pd.concat(
        [perf_vf3_success, aborted_vf3_success], ignore_index=True
    )

    vf2_mean = (
        all_vf2_success["vf2_mean"].mean()
        if not all_vf2_success.empty
        else float("nan")
    )
    vf3_mean = (
        all_vf3_success["vf3_mean"].mean()
        if not all_vf3_success.empty
        else float("nan")
    )
    vf2_median = (
        all_vf2_success["vf2_mean"].median()
        if not all_vf2_success.empty
        else float("nan")
    )
    vf3_median = (
        all_vf3_success["vf3_mean"].median()
        if not all_vf3_success.empty
        else float("nan")
    )
    vf2_std = (
        all_vf2_success["vf2_mean"].std(ddof=1)
        if len(all_vf2_success) > 1
        else float("nan")
    )
    vf3_std = (
        all_vf3_success["vf3_mean"].std(ddof=1)
        if len(all_vf3_success) > 1
        else float("nan")
    )

    print("Statistics:")
    print(f"  Total completed VF2 runs: {vf2_success_total}")
    print(f"  Total completed VF3 runs: {vf3_success_total}")
    print(f"  Total aborted VF2 runs: {vf2_fail_total}")
    print(f"  Total aborted VF3 runs: {vf3_fail_total}")
    print(
        f"  VF3 faster than VF2 (+ VF3 completed/VF2 aborted): {vf3_faster} (+{vf3_success_vf2_aborted})"
    )
    print(
        f"  VF2 faster than VF3 (+ VF2 completed/ VF3 aborted): {vf2_faster} (+{vf2_success_vf3_aborted})"
    )
    print(f"  Mean completed VF2 run: {vf2_mean:.6f}")
    print(f"  Mean completed VF3 run: {vf3_mean:.6f}")
    print(f"  Median completed VF2 run: {vf2_median:.6f}")
    print(f"  Median completed VF3 run: {vf3_median:.6f}")
    print(f"  Std dev VF2 (completed): {vf2_std:.6f}")
    print(f"  Std dev VF3 (completed): {vf3_std:.6f}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot VF2/VF3 timings from perf_results.csv"
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path(__file__).parent / "results" / "perf_results.csv",
        help="Path to perf_results.csv",
    )
    parser.add_argument(
        "--aborted-csv",
        type=Path,
        default=Path(__file__).parent / "results" / "aborted_results.csv",
        help="Path to aborted_results.csv",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).parent / "results" / "vf2_vf3.pdf",
        help="Output plot path (image)",
    )
    args = parser.parse_args()

    configure_plot_style()
    perf_df = pd.read_csv(args.csv)
    perf_df = add_density_columns(perf_df)
    if args.aborted_csv.exists():
        aborted_df = pd.read_csv(args.aborted_csv)
        aborted_df = add_density_columns(aborted_df)
    else:
        aborted_df = pd.DataFrame(columns=perf_df.columns)

    required_cols = {"vf2_mean", "vf3_mean", "vf2_matches", "vf3_matches"}
    missing_required = required_cols - set(perf_df.columns)
    if missing_required:
        raise ValueError(f"CSV is missing required columns: {missing_required}")

    compute_stats(perf_df, aborted_df)

    sorted_df = sort_dataframe(perf_df)
    plot_results(sorted_df, args.output)


if __name__ == "__main__":
    main()
