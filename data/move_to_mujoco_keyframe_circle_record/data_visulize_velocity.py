#!/usr/bin/env python3
"""
Visualize joint velocities for all 7 joints.
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


def compute_derivative(values, time):
    return np.gradient(values, time)


def main():
    parser = argparse.ArgumentParser(description="Visualize joint velocities.")
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

    # Create figure with 7 subplots
    fig, axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    fig.suptitle("Joint Velocities", fontsize=14, fontweight="bold")

    # Use a colormap for different joints
    colors = plt.cm.viridis(np.linspace(0.1, 0.9, 7))

    for i in range(7):
        ax = axes[i]
        dq_actual_col = f"dq_actual_{i+1}"
        dq_desired_col = f"dq_desired_{i+1}"

        if dq_actual_col in df.columns:
            dq_actual = df[dq_actual_col].values
        else:
            q_actual_col = f"q_actual_{i+1}"
            if q_actual_col not in df.columns:
                raise KeyError(f"Missing joint angle column: {q_actual_col}")
            dq_actual = compute_derivative(df[q_actual_col].values, time)

        dq_desired = None
        if dq_desired_col in df.columns:
            dq_desired = df[dq_desired_col].values
        else:
            q_desired_col = f"q_desired_{i+1}"
            if q_desired_col in df.columns:
                dq_desired = compute_derivative(df[q_desired_col].values, time)

        if dq_desired is not None:
            ax.plot(time, dq_desired, color="#e74c3c", linewidth=1.2,
                    alpha=0.9, linestyle="--", label="Desired")
        ax.plot(time, dq_actual, color=colors[i], linewidth=1.5, alpha=0.9, label="Actual")

        ax.set_ylabel(f"Joint {i+1}\n(rad/s)", fontsize=10)
        ax.grid(True, alpha=0.3)
        if i == 0 and dq_desired is not None:
            ax.legend(loc="upper right", fontsize=8)

        # Display max and min velocity
        max_vel = np.max(dq_actual)
        min_vel = np.min(dq_actual)
        ax.text(0.02, 0.95, f"Max: {max_vel:.4f}\nMin: {min_vel:.4f}",
                transform=ax.transAxes, fontsize=8, verticalalignment="top",
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
