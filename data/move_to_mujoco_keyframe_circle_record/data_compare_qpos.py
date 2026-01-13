#!/usr/bin/env python3
"""
Compare desired and actual joint angles for all 7 joints.
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
    fig.suptitle('Joint Angles: Desired vs Actual', fontsize=14, fontweight='bold')
    
    colors = {'desired': '#e74c3c', 'actual': '#3498db'}
    
    for i in range(7):
        ax = axes[i]
        q_desired = df[f'q_desired_{i+1}'].values
        q_actual = df[f'q_actual_{i+1}'].values
        
        ax.plot(time, q_desired, color=colors['desired'], linewidth=1.5, 
                label='Desired', alpha=0.9)
        ax.plot(time, q_actual, color=colors['actual'], linewidth=1.5, 
                label='Actual', linestyle='--', alpha=0.9)
        
        ax.set_ylabel(f'Joint {i+1}\n(rad)', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)
        
        # Calculate and display max error
        max_error = np.max(np.abs(q_desired - q_actual))
        ax.text(0.02, 0.95, f'Max Error: {max_error:.6f} rad', 
                transform=ax.transAxes, fontsize=8, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    axes[-1].set_xlabel('Time (s)', fontsize=12)
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(script_dir, 'compare_qpos.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Figure saved to: {output_path}")
    
    plt.show()

if __name__ == '__main__':
    main()
