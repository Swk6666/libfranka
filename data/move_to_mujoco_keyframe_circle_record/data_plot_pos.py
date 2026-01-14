#!/usr/bin/env python3
"""
Visualize cartesian position (X, Y, Z) trajectory.
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
    x = df['x'].values
    y = df['y'].values
    z = df['z'].values
    
    # Create figure with 3 subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle('Cartesian Position Trajectory', fontsize=14, fontweight='bold')
    
    # X position
    axes[0].plot(time, x, color='#e74c3c', linewidth=1.5, alpha=0.9, label='X position')
    axes[0].set_ylabel('X (m)', fontsize=11)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='upper right')
    max_x = np.max(x)
    min_x = np.min(x)
    axes[0].text(0.02, 0.95, f'Max: {max_x:.4f} m\nMin: {min_x:.4f} m', 
                transform=axes[0].transAxes, fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.5))
    
    # Y position
    axes[1].plot(time, y, color='#2ecc71', linewidth=1.5, alpha=0.9, label='Y position')
    axes[1].set_ylabel('Y (m)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='upper right')
    max_y = np.max(y)
    min_y = np.min(y)
    axes[1].text(0.02, 0.95, f'Max: {max_y:.4f} m\nMin: {min_y:.4f} m', 
                transform=axes[1].transAxes, fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.5))
    
    # Z position
    axes[2].plot(time, z, color='#3498db', linewidth=1.5, alpha=0.9, label='Z position')
    axes[2].set_ylabel('Z (m)', fontsize=11)
    axes[2].set_xlabel('Time (s)', fontsize=12)
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc='upper right')
    max_z = np.max(z)
    min_z = np.min(z)
    axes[2].text(0.02, 0.95, f'Max: {max_z:.4f} m\nMin: {min_z:.4f} m', 
                transform=axes[2].transAxes, fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(script_dir, 'visualize_position.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Figure saved to: {output_path}")
    
    plt.show()

if __name__ == '__main__':
    main()
