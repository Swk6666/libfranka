#!/usr/bin/env python3
"""
Visualize cartesian position (X, Y, Z) trajectory.
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os


def resolve_csv_path(script_dir, input_path):
    if input_path:
        return os.path.abspath(input_path)
    candidates = [
        os.path.join(script_dir, "..", "circle_trajectory_impedance", "circle_trajectory_data_impedance.csv"),
        os.path.join(script_dir, "trajectory_data.csv"),
    ]
    for path in candidates:
        if os.path.exists(path):
            return os.path.abspath(path)
    return os.path.abspath(candidates[0])


def get_column(df, *names):
    for name in names:
        if name in df.columns:
            return df[name].values
    return None


def main():
    parser = argparse.ArgumentParser(description="Plot Cartesian position trajectory.")
    parser.add_argument("--input", default=None, help="Path to trajectory CSV")
    args = parser.parse_args()

    # Load data
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = resolve_csv_path(script_dir, args.input)
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    print(f"Loading data from: {csv_path}")
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} data points")

    time = df["time"].values
    time = time - time[0]

    desired = {
        "x": get_column(df, "desired_x", "ee_desired_x"),
        "y": get_column(df, "desired_y", "ee_desired_y"),
        "z": get_column(df, "desired_z", "ee_desired_z"),
    }
    actual = {
        "x": get_column(df, "actual_x", "ee_actual_x", "x"),
        "y": get_column(df, "actual_y", "ee_actual_y", "y"),
        "z": get_column(df, "actual_z", "ee_actual_z", "z"),
    }
    commanded = {
        "x": get_column(df, "commanded_x", "ee_commanded_x"),
        "y": get_column(df, "commanded_y", "ee_commanded_y"),
        "z": get_column(df, "commanded_z", "ee_commanded_z"),
    }

    if actual["x"] is None or actual["y"] is None or actual["z"] is None:
        raise KeyError("Missing Cartesian position columns in CSV")

    # Create figure with 3 subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("Cartesian Position Trajectory", fontsize=14, fontweight="bold")

    colors = {"desired": "#e74c3c", "actual": "#3498db", "commanded": "#2ecc71"}
    axis_names = ["x", "y", "z"]
    axis_labels = ["X", "Y", "Z"]

    for i, (axis_name, axis_label) in enumerate(zip(axis_names, axis_labels)):
        ax = axes[i]
        actual_series = actual[axis_name]
        desired_series = desired[axis_name]
        commanded_series = commanded[axis_name]

        if desired_series is not None:
            ax.plot(time, desired_series, color=colors["desired"], linewidth=1.5,
                    alpha=0.9, label="Desired")
        ax.plot(time, actual_series, color=colors["actual"], linewidth=1.5,
                alpha=0.9, label="Actual")
        if commanded_series is not None:
            ax.plot(time, commanded_series, color=colors["commanded"], linewidth=1.0,
                    alpha=0.8, linestyle=":", label="Commanded")

        ax.set_ylabel(f"{axis_label} (m)", fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")

        max_val = np.max(actual_series)
        min_val = np.min(actual_series)
        if desired_series is not None:
            max_error = np.max(np.abs(desired_series - actual_series)) * 1000.0
            stats = f"Max: {max_val:.4f} m\nMin: {min_val:.4f} m\nMax Err: {max_error:.2f} mm"
        else:
            stats = f"Max: {max_val:.4f} m\nMin: {min_val:.4f} m"
        ax.text(0.02, 0.95, stats,
                transform=ax.transAxes, fontsize=9, verticalalignment="top",
                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

    axes[-1].set_xlabel("Time (s)", fontsize=12)

    plt.tight_layout()

    # Save figure
    output_path = os.path.join(script_dir, "visualize_position.png")
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"Figure saved to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
