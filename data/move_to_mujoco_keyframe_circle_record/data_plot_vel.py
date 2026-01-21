#!/usr/bin/env python3
"""
Visualize cartesian velocity (VX, VY, VZ) trajectory.
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


def compute_derivative(values, time):
    return np.gradient(values, time)


def main():
    parser = argparse.ArgumentParser(description="Plot Cartesian velocity trajectory.")
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

    desired_pos = {
        "x": get_column(df, "desired_x", "ee_desired_x"),
        "y": get_column(df, "desired_y", "ee_desired_y"),
        "z": get_column(df, "desired_z", "ee_desired_z"),
    }
    actual_pos = {
        "x": get_column(df, "actual_x", "ee_actual_x", "x"),
        "y": get_column(df, "actual_y", "ee_actual_y", "y"),
        "z": get_column(df, "actual_z", "ee_actual_z", "z"),
    }
    direct_vel = {
        "x": get_column(df, "vx"),
        "y": get_column(df, "vy"),
        "z": get_column(df, "vz"),
    }

    # Create figure with 3 subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("Cartesian Velocity Trajectory", fontsize=14, fontweight="bold")

    colors = {"desired": "#e74c3c", "actual": "#3498db"}
    axis_names = ["x", "y", "z"]
    axis_labels = ["VX", "VY", "VZ"]

    for i, (axis_name, axis_label) in enumerate(zip(axis_names, axis_labels)):
        ax = axes[i]

        desired_vel = None
        if desired_pos[axis_name] is not None:
            desired_vel = compute_derivative(desired_pos[axis_name], time)

        if direct_vel[axis_name] is not None:
            actual_vel = direct_vel[axis_name]
        elif actual_pos[axis_name] is not None:
            actual_vel = compute_derivative(actual_pos[axis_name], time)
        else:
            raise KeyError(f"Missing data for {axis_name}-axis velocity")

        if desired_vel is not None:
            ax.plot(time, desired_vel, color=colors["desired"], linewidth=1.5,
                    alpha=0.9, label="Desired")
        ax.plot(time, actual_vel, color=colors["actual"], linewidth=1.5,
                alpha=0.9, label="Actual")

        ax.set_ylabel(f"{axis_label} (m/s)", fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")

        max_val = np.max(actual_vel)
        min_val = np.min(actual_vel)
        ax.text(0.02, 0.95, f"Max: {max_val:.4f} m/s\nMin: {min_val:.4f} m/s",
                transform=ax.transAxes, fontsize=9, verticalalignment="top",
                bbox=dict(boxstyle="round", facecolor="lightblue", alpha=0.5))

    axes[-1].set_xlabel("Time (s)", fontsize=12)

    plt.tight_layout()

    # Save figure
    output_path = os.path.join(script_dir, "visualize_velocity.png")
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"Figure saved to: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
