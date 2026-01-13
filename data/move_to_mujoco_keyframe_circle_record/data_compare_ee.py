#!/usr/bin/env python3
"""
Compare desired and actual end-effector positions (X, Y, Z).
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

def main():
    # Load data from same directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(script_dir, 'trajectory_data.csv')
    
    print(f"Loading data from: {csv_path}")
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} data points")
    
    time = df['time'].values
    
    # Create figure with 3 subplots (X, Y, Z)
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('End-Effector Position: Desired vs Actual', fontsize=14, fontweight='bold')
    
    colors = {'desired': '#e74c3c', 'actual': '#3498db'}
    axis_labels = ['X', 'Y', 'Z']
    axis_names = ['x', 'y', 'z']
    
    for i, (label, name) in enumerate(zip(axis_labels, axis_names)):
        ax = axes[i]
        
        ee_desired = df[f'ee_desired_{name}'].values
        ee_actual = df[f'ee_actual_{name}'].values
        
        ax.plot(time, ee_desired, color=colors['desired'], linewidth=1.5, 
                label='Desired', alpha=0.9)
        ax.plot(time, ee_actual, color=colors['actual'], linewidth=1.5, 
                label='Actual', linestyle='--', alpha=0.9)
        
        ax.set_ylabel(f'{label} Position\n(m)', fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=9)
        
        # Calculate and display max error
        max_error = np.max(np.abs(ee_desired - ee_actual))
        rmse = np.sqrt(np.mean((ee_desired - ee_actual)**2))
        ax.text(0.02, 0.95, f'Max Error: {max_error*1000:.3f} mm\nRMSE: {rmse*1000:.3f} mm', 
                transform=ax.transAxes, fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    axes[-1].set_xlabel('Time (s)', fontsize=12)
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(script_dir, 'compare_ee.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Figure saved to: {output_path}")
    
    # Also create a 3D trajectory comparison plot
    fig2 = plt.figure(figsize=(10, 8))
    ax3d = fig2.add_subplot(111, projection='3d')
    
    ee_desired_x = df['ee_desired_x'].values
    ee_desired_y = df['ee_desired_y'].values
    ee_desired_z = df['ee_desired_z'].values
    ee_actual_x = df['ee_actual_x'].values
    ee_actual_y = df['ee_actual_y'].values
    ee_actual_z = df['ee_actual_z'].values
    
    ax3d.plot(ee_desired_x, ee_desired_y, ee_desired_z, 
              color=colors['desired'], linewidth=2, label='Desired', alpha=0.9)
    ax3d.plot(ee_actual_x, ee_actual_y, ee_actual_z, 
              color=colors['actual'], linewidth=2, label='Actual', linestyle='--', alpha=0.9)
    
    # Mark start and end points
    ax3d.scatter([ee_desired_x[0]], [ee_desired_y[0]], [ee_desired_z[0]], 
                 c='green', s=100, marker='o', label='Start')
    ax3d.scatter([ee_desired_x[-1]], [ee_desired_y[-1]], [ee_desired_z[-1]], 
                 c='red', s=100, marker='s', label='End')
    
    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m)')
    ax3d.set_title('3D End-Effector Trajectory: Desired vs Actual')
    ax3d.legend()
    
    # Save 3D figure
    output_path_3d = os.path.join(script_dir, 'compare_ee_3d.png')
    plt.savefig(output_path_3d, dpi=150, bbox_inches='tight')
    print(f"3D Figure saved to: {output_path_3d}")
    
    plt.show()

if __name__ == '__main__':
    main()
