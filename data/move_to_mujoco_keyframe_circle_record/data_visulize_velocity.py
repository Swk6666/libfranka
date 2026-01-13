#!/usr/bin/env python3
"""
Visualize actual joint velocities for all 7 joints.
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
    
    # Create figure with 7 subplots
    fig, axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    fig.suptitle('Actual Joint Velocities', fontsize=14, fontweight='bold')
    
    # Use a colormap for different joints
    colors = plt.cm.viridis(np.linspace(0.1, 0.9, 7))
    
    for i in range(7):
        ax = axes[i]
        dq_actual = df[f'dq_actual_{i+1}'].values
        
        ax.plot(time, dq_actual, color=colors[i], linewidth=1.5, alpha=0.9)
        
        ax.set_ylabel(f'Joint {i+1}\n(rad/s)', fontsize=10)
        ax.grid(True, alpha=0.3)
        
        # Display max and min velocity
        max_vel = np.max(dq_actual)
        min_vel = np.min(dq_actual)
        ax.text(0.02, 0.95, f'Max: {max_vel:.4f}\nMin: {min_vel:.4f}', 
                transform=ax.transAxes, fontsize=8, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    
    axes[-1].set_xlabel('Time (s)', fontsize=12)
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(script_dir, 'visualize_velocity.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Figure saved to: {output_path}")
    
    plt.show()

if __name__ == '__main__':
    main()
