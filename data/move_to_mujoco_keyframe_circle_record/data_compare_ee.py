#!/usr/bin/env python3
"""
Compare desired and actual end-effector positions (X, Y, Z).
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
    parser = argparse.ArgumentParser(description="Compare desired vs actual end-effector positions.")
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

    # Create figure with 3 subplots (X, Y, Z)
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("End-Effector Position: Desired vs Actual", fontsize=14, fontweight="bold")

    colors = {"desired": "#e74c3c", "actual": "#3498db", "commanded": "#2ecc71"}
    axis_labels = ["X", "Y", "Z"]
    axis_names = ["x", "y", "z"]

    for i, (label, name) in enumerate(zip(axis_labels, axis_names)):
        ax = axes[i]

        ee_desired = get_column(df, f"desired_{name}", f"ee_desired_{name}")
        ee_actual = get_column(df, f"actual_{name}", f"ee_actual_{name}")
        ee_commanded = get_column(df, f"commanded_{name}", f"ee_commanded_{name}")
        if ee_desired is None or ee_actual is None:
            raise KeyError(f"Missing columns for {name} position in CSV")

        ax.plot(time, ee_desired, color=colors["desired"], linewidth=1.5,
                label="Desired", alpha=0.9)
        ax.plot(time, ee_actual, color=colors["actual"], linewidth=1.5,
                label="Actual", linestyle="--", alpha=0.9)
        if ee_commanded is not None:
            ax.plot(time, ee_commanded, color=colors["commanded"], linewidth=1.0,
                    label="Commanded", linestyle=":", alpha=0.8)

        ax.set_ylabel(f"{label} Position\n(m)", fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=9)

        # Calculate and display error stats
        max_error = np.max(np.abs(ee_desired - ee_actual))
        rmse = np.sqrt(np.mean((ee_desired - ee_actual) ** 2))
        ax.text(0.02, 0.95, f"Max Error: {max_error*1000:.3f} mm\nRMSE: {rmse*1000:.3f} mm",
                transform=ax.transAxes, fontsize=9, verticalalignment="top",
                bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

    axes[-1].set_xlabel("Time (s)", fontsize=12)

    plt.tight_layout()

    # Save figure
    output_path = os.path.join(script_dir, "compare_ee.png")
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"Figure saved to: {output_path}")

    # Also create a 3D trajectory comparison plot
    fig2 = plt.figure(figsize=(10, 8))
    ax3d = fig2.add_subplot(111, projection="3d")

    ee_desired_x = get_column(df, "desired_x", "ee_desired_x")
    ee_desired_y = get_column(df, "desired_y", "ee_desired_y")
    ee_desired_z = get_column(df, "desired_z", "ee_desired_z")
    ee_actual_x = get_column(df, "actual_x", "ee_actual_x")
    ee_actual_y = get_column(df, "actual_y", "ee_actual_y")
    ee_actual_z = get_column(df, "actual_z", "ee_actual_z")
    ee_commanded_x = get_column(df, "commanded_x", "ee_commanded_x")
    ee_commanded_y = get_column(df, "commanded_y", "ee_commanded_y")
    ee_commanded_z = get_column(df, "commanded_z", "ee_commanded_z")

    ax3d.plot(ee_desired_x, ee_desired_y, ee_desired_z,
              color=colors["desired"], linewidth=2, label="Desired", alpha=0.9)
    ax3d.plot(ee_actual_x, ee_actual_y, ee_actual_z,
              color=colors["actual"], linewidth=2, label="Actual", linestyle="--", alpha=0.9)
    if ee_commanded_x is not None:
        ax3d.plot(ee_commanded_x, ee_commanded_y, ee_commanded_z,
                  color=colors["commanded"], linewidth=1, label="Commanded", linestyle=":", alpha=0.8)

    # Mark start and end points
    ax3d.scatter([ee_desired_x[0]], [ee_desired_y[0]], [ee_desired_z[0]],
                 c="green", s=100, marker="o", label="Start")
    ax3d.scatter([ee_desired_x[-1]], [ee_desired_y[-1]], [ee_desired_z[-1]],
                 c="red", s=100, marker="s", label="End")

    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.set_title("3D End-Effector Trajectory: Desired vs Actual")
    ax3d.legend()

    # Save 3D figure
    output_path_3d = os.path.join(script_dir, "compare_ee_3d.png")
    plt.savefig(output_path_3d, dpi=150, bbox_inches="tight")
    print(f"3D Figure saved to: {output_path_3d}")

    plt.show()


if __name__ == "__main__":
    main()
