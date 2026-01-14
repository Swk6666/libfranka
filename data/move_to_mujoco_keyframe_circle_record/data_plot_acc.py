#!/usr/bin/env python3
"""
Visualize cartesian acceleration (AX, AY, AZ) trajectory.
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
    ax = df['ax'].values
    ay = df['ay'].values
    az = df['az'].values
    
    # Create figure with 3 subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle('Cartesian Acceleration Trajectory', fontsize=14, fontweight='bold')
    
    # AX acceleration
    axes[0].plot(time, ax, color='#e74c3c', linewidth=1.5, alpha=0.9, label='AX acceleration')
    axes[0].set_ylabel('AX (m/s²)', fontsize=11)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='upper right')
    max_ax = np.max(ax)
    min_ax = np.min(ax)
    axes[0].text(0.02, 0.95, f'Max: {max_ax:.4f} m/s²\nMin: {min_ax:.4f} m/s²', 
                transform=axes[0].transAxes, fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.5))
    
    # AY acceleration
    axes[1].plot(time, ay, color='#2ecc71', linewidth=1.5, alpha=0.9, label='AY acceleration')
    axes[1].set_ylabel('AY (m/s²)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='upper right')
    max_ay = np.max(ay)
    min_ay = np.min(ay)
    axes[1].text(0.02, 0.95, f'Max: {max_ay:.4f} m/s²\nMin: {min_ay:.4f} m/s²', 
                transform=axes[1].transAxes, fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.5))
    
    # AZ acceleration
    axes[2].plot(time, az, color='#3498db', linewidth=1.5, alpha=0.9, label='AZ acceleration')
    axes[2].set_ylabel('AZ (m/s²)', fontsize=11)
    axes[2].set_xlabel('Time (s)', fontsize=12)
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc='upper right')
    max_az = np.max(az)
    min_az = np.min(az)
    axes[2].text(0.02, 0.95, f'Max: {max_az:.4f} m/s²\nMin: {min_az:.4f} m/s²', 
                transform=axes[2].transAxes, fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(script_dir, 'visualize_acceleration.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Figure saved to: {output_path}")
    
    plt.show()

if __name__ == '__main__':
    main()
