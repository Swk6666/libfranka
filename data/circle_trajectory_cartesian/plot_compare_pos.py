#!/usr/bin/env python3
"""
Compare desired and actual Cartesian positions for a circle trajectory.
"""

import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
import os

try:
    import pandas as pd
except ImportError:  # Keep optional dependency for portability.
    pd = None


def load_csv(path):
    if pd is not None:
        return pd.read_csv(path)

    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        rows = list(reader)
        if not rows:
            raise ValueError("CSV file is empty.")
        data = {}
        for key in reader.fieldnames:
            data[key] = np.array([float(row[key]) for row in rows], dtype=float)
    return data


def get_series(data, key):
    series = data[key]
    if hasattr(series, "to_numpy"):
        return series.to_numpy()
    return np.asarray(series, dtype=float)


def get_columns(data):
    if hasattr(data, "columns"):
        return list(data.columns)
    return list(data.keys())


def require_columns(data, columns):
    missing = [name for name in columns if name not in get_columns(data)]
    if missing:
        missing_str = ", ".join(missing)
        raise KeyError(f"Missing columns in CSV: {missing_str}")


def main():
    parser = argparse.ArgumentParser(
        description="Plot desired vs actual Cartesian positions from a trajectory CSV."
    )
    parser.add_argument(
        "--input",
        default=None,
        help="Path to CSV file (default: circle_trajectory_data_cartesian.csv next to script)",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output image path (default: compare_pos_cartesian.png next to script)",
    )
    parser.add_argument(
        "--no-commanded",
        action="store_true",
        help="Do not plot commanded positions even if columns are present",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not display the figure window",
    )
    args = parser.parse_args()

    # Load data from same directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = args.input or os.path.join(script_dir, "circle_trajectory_data_cartesian.csv")
    
    print(f"Loading data from: {csv_path}")
    if not os.path.exists(csv_path):
        print(f"Error: File not found: {csv_path}")
        print("Please run the robot program first to generate the data file.")
        return
    
    data = load_csv(csv_path)
    required = [
        "timestamp",
        "desired_x",
        "desired_y",
        "desired_z",
        "actual_x",
        "actual_y",
        "actual_z",
    ]
    require_columns(data, required)

    num_points = len(get_series(data, "timestamp"))
    print(f"Loaded {num_points} data points")
    time_raw = get_series(data, "timestamp")
    time = time_raw - time_raw[0]
    print(f"Time range: {time.min():.2f}s to {time.max():.2f}s")
    
    # Create figure with multiple subplots
    fig = plt.figure(figsize=(16, 10))
    
    # Color scheme
    colors = {
        'desired': '#e74c3c',    # Red
        'actual': '#3498db',     # Blue
        'commanded': '#2ecc71'   # Green
    }
    
    # ========== 1. X, Y, Z Position vs Time (3 subplots) ==========
    axes_xyz = []
    for i, coord in enumerate(['x', 'y', 'z']):
        ax = plt.subplot(3, 3, i+1)
        axes_xyz.append(ax)
        
        desired = get_series(data, f"desired_{coord}")
        actual = get_series(data, f"actual_{coord}")
        commanded = None
        if not args.no_commanded:
            commanded_key = f"commanded_{coord}"
            if commanded_key in get_columns(data):
                commanded = get_series(data, commanded_key)
        
        ax.plot(time, desired, color=colors['desired'], linewidth=2, 
                label='Desired', alpha=0.9)
        ax.plot(time, actual, color=colors['actual'], linewidth=1.5, 
                label='Actual', linestyle='--', alpha=0.9)
        if commanded is not None:
            ax.plot(time, commanded, color=colors['commanded'], linewidth=1, 
                    label='Commanded', linestyle=':', alpha=0.7)
        
        ax.set_ylabel(f'{coord.upper()} Position (m)', fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=9)
        ax.set_title(f'{coord.upper()}-axis Position', fontsize=12, fontweight='bold')
        
        # Calculate and display statistics
        error = np.abs(desired - actual)
        max_error = np.max(error)
        mean_error = np.mean(error)
        ax.text(0.02, 0.98, f'Max Error: {max_error*1000:.2f} mm\nMean Error: {mean_error*1000:.2f} mm', 
                transform=ax.transAxes, fontsize=8, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))
    
    axes_xyz[-1].set_xlabel('Time (s)', fontsize=11)
    
    # ========== 2. Error plots (3 subplots) ==========
    for i, coord in enumerate(['x', 'y', 'z']):
        ax = plt.subplot(3, 3, i+4)
        
        desired = get_series(data, f"desired_{coord}")
        actual = get_series(data, f"actual_{coord}")
        error = (desired - actual) * 1000  # Convert to mm
        
        ax.plot(time, error, color='#e67e22', linewidth=1.5, alpha=0.9)
        ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.5)
        
        ax.set_ylabel(f'{coord.upper()} Error (mm)', fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.set_title(f'{coord.upper()}-axis Tracking Error', fontsize=12, fontweight='bold')
        
        # Statistics
        rms_error = np.sqrt(np.mean(error**2))
        ax.text(0.02, 0.98, f'RMS: {rms_error:.3f} mm', 
                transform=ax.transAxes, fontsize=8, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
    
    plt.subplot(3, 3, 6).set_xlabel('Time (s)', fontsize=11)
    
    # ========== 3. 3D Trajectory Visualization ==========
    ax_3d = plt.subplot(3, 3, 7, projection='3d')
    
    desired_x = get_series(data, "desired_x")
    desired_y = get_series(data, "desired_y")
    desired_z = get_series(data, "desired_z")
    actual_x = get_series(data, "actual_x")
    actual_y = get_series(data, "actual_y")
    actual_z = get_series(data, "actual_z")
    
    ax_3d.plot(desired_x, desired_y, desired_z, color=colors['desired'], 
               linewidth=2, label='Desired', alpha=0.9)
    ax_3d.plot(actual_x, actual_y, actual_z, color=colors['actual'], 
               linewidth=1.5, label='Actual', linestyle='--', alpha=0.9)
    
    # Mark start and end points
    ax_3d.scatter(desired_x[0], desired_y[0], desired_z[0], 
                  color='green', s=100, marker='o', label='Start', zorder=5)
    ax_3d.scatter(desired_x[-1], desired_y[-1], desired_z[-1], 
                  color='red', s=100, marker='s', label='End', zorder=5)
    
    ax_3d.set_xlabel('X (m)', fontsize=10)
    ax_3d.set_ylabel('Y (m)', fontsize=10)
    ax_3d.set_zlabel('Z (m)', fontsize=10)
    ax_3d.set_title('3D Trajectory', fontsize=12, fontweight='bold')
    ax_3d.legend(fontsize=8)
    ax_3d.grid(True, alpha=0.3)
    
    # ========== 4. XY Plane View (Circle) ==========
    ax_xy = plt.subplot(3, 3, 8)
    
    ax_xy.plot(desired_x, desired_y, color=colors['desired'], 
               linewidth=2, label='Desired', alpha=0.9)
    ax_xy.plot(actual_x, actual_y, color=colors['actual'], 
               linewidth=1.5, label='Actual', linestyle='--', alpha=0.9)
    
    ax_xy.scatter(desired_x[0], desired_y[0], color='green', s=100, 
                  marker='o', label='Start', zorder=5)
    ax_xy.scatter(desired_x[-1], desired_y[-1], color='red', s=100, 
                  marker='s', label='End', zorder=5)
    
    ax_xy.set_xlabel('X (m)', fontsize=11)
    ax_xy.set_ylabel('Y (m)', fontsize=11)
    ax_xy.set_title('XY Plane (Top View)', fontsize=12, fontweight='bold')
    ax_xy.legend(fontsize=8)
    ax_xy.grid(True, alpha=0.3)
    ax_xy.axis('equal')
    
    # ========== 5. Total Error Norm ==========
    ax_norm = plt.subplot(3, 3, 9)
    
    if "error_norm" in get_columns(data):
        error_norm = get_series(data, "error_norm") * 1000  # Convert to mm
    else:
        error_norm = np.sqrt(
            (desired_x - actual_x) ** 2
            + (desired_y - actual_y) ** 2
            + (desired_z - actual_z) ** 2
        ) * 1000
    
    ax_norm.plot(time, error_norm, color='#9b59b6', linewidth=2, alpha=0.9)
    ax_norm.fill_between(time, 0, error_norm, color='#9b59b6', alpha=0.2)
    
    ax_norm.set_xlabel('Time (s)', fontsize=11)
    ax_norm.set_ylabel('Position Error (mm)', fontsize=11)
    ax_norm.set_title('Total Position Error Norm', fontsize=12, fontweight='bold')
    ax_norm.grid(True, alpha=0.3)
    
    # Statistics
    max_error = np.max(error_norm)
    mean_error = np.mean(error_norm)
    rms_error = np.sqrt(np.mean(error_norm**2))
    ax_norm.text(0.98, 0.98, 
                 f'Max: {max_error:.3f} mm\nMean: {mean_error:.3f} mm\nRMS: {rms_error:.3f} mm', 
                 transform=ax_norm.transAxes, fontsize=9, 
                 verticalalignment='top', horizontalalignment='right',
                 bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
    
    # Main title
    fig.suptitle('Circle Trajectory - Cartesian Position Control Analysis', 
                 fontsize=16, fontweight='bold', y=0.995)
    
    plt.tight_layout(rect=[0, 0, 1, 0.99])
    
    # Save figure
    output_path = args.output or os.path.join(script_dir, "compare_pos_cartesian.png")
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nFigure saved to: {output_path}")
    
    # Print summary statistics
    print("\n" + "="*60)
    print("TRACKING PERFORMANCE SUMMARY (Cartesian Control)")
    print("="*60)
    print(f"Total data points: {num_points}")
    print(f"Duration: {time[-1]:.2f} seconds")
    print(f"\nPosition Error (3D norm):")
    print(f"  Maximum:  {np.max(error_norm):.3f} mm")
    print(f"  Mean:     {np.mean(error_norm):.3f} mm")
    print(f"  RMS:      {np.sqrt(np.mean(error_norm**2)):.3f} mm")
    print(f"  Std Dev:  {np.std(error_norm):.3f} mm")
    
    print(f"\nPer-axis errors:")
    for coord in ['x', 'y', 'z']:
        error = np.abs(
            get_series(data, f"desired_{coord}") - get_series(data, f"actual_{coord}")
        ) * 1000
        print(f"  {coord.upper()}-axis: Max={np.max(error):.3f} mm, "
              f"Mean={np.mean(error):.3f} mm, RMS={np.sqrt(np.mean(error**2)):.3f} mm")
    print("="*60)
    
    if not args.no_show:
        plt.show()

if __name__ == '__main__':
    main()
