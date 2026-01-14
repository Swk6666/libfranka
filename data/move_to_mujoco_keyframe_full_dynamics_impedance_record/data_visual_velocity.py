#!/usr/bin/env python3
"""
Visualize desired and actual joint velocities for all 7 joints.
Full dynamics impedance control data visualization.
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
    
    # Create figure with 2 rows x 4 columns subplots
    fig, axes = plt.subplots(2, 4, figsize=(16, 8), sharex=True)
    fig.suptitle('Joint Velocities: Desired vs Actual\n(Full Dynamics Impedance Control)', 
                 fontsize=14, fontweight='bold')
    
    colors = {'desired': '#e74c3c', 'actual': '#3498db'}
    
    # Flatten axes array for easier indexing
    axes_flat = axes.flatten()
    
    for i in range(7):
        ax = axes_flat[i]
        dq_desired = df[f'dq_desired_{i+1}'].values
        dq_actual = df[f'dq_actual_{i+1}'].values
        
        ax.plot(time, dq_desired, color=colors['desired'], linewidth=1.5, 
                label='Desired', alpha=0.9)
        ax.plot(time, dq_actual, color=colors['actual'], linewidth=1.5, 
                label='Actual', linestyle='--', alpha=0.9)
        
        ax.set_ylabel(f'Joint {i+1} (rad/s)', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        
        # Add x-label to bottom row
        if i >= 4:  # Bottom row (indices 4, 5, 6, 7)
            ax.set_xlabel('Time (s)', fontsize=10)
    
    # Hide the last subplot (index 7) as we only have 7 joints
    axes_flat[7].axis('off')
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(script_dir, 'visualize_velocity.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Figure saved to: {output_path}")
    
    plt.show()

if __name__ == '__main__':
    main()
