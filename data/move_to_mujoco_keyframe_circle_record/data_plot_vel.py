#!/usr/bin/env python3
"""
Visualize cartesian velocity (VX, VY, VZ) trajectory.
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
    vx = df['vx'].values
    vy = df['vy'].values
    vz = df['vz'].values
    
    # Create figure with 3 subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle('Cartesian Velocity Trajectory', fontsize=14, fontweight='bold')
    
    # VX velocity
    axes[0].plot(time, vx, color='#e74c3c', linewidth=1.5, alpha=0.9, label='VX velocity')
    axes[0].set_ylabel('VX (m/s)', fontsize=11)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='upper right')
    max_vx = np.max(vx)
    min_vx = np.min(vx)
    axes[0].text(0.02, 0.95, f'Max: {max_vx:.4f} m/s\nMin: {min_vx:.4f} m/s', 
                transform=axes[0].transAxes, fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.5))
    
    # VY velocity
    axes[1].plot(time, vy, color='#2ecc71', linewidth=1.5, alpha=0.9, label='VY velocity')
    axes[1].set_ylabel('VY (m/s)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='upper right')
    max_vy = np.max(vy)
    min_vy = np.min(vy)
    axes[1].text(0.02, 0.95, f'Max: {max_vy:.4f} m/s\nMin: {min_vy:.4f} m/s', 
                transform=axes[1].transAxes, fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.5))
    
    # VZ velocity
    axes[2].plot(time, vz, color='#3498db', linewidth=1.5, alpha=0.9, label='VZ velocity')
    axes[2].set_ylabel('VZ (m/s)', fontsize=11)
    axes[2].set_xlabel('Time (s)', fontsize=12)
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc='upper right')
    max_vz = np.max(vz)
    min_vz = np.min(vz)
    axes[2].text(0.02, 0.95, f'Max: {max_vz:.4f} m/s\nMin: {min_vz:.4f} m/s', 
                transform=axes[2].transAxes, fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(script_dir, 'visualize_velocity.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Figure saved to: {output_path}")
    
    plt.show()

if __name__ == '__main__':
    main()
