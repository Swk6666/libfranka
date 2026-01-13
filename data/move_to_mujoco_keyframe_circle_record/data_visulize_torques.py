#!/usr/bin/env python3
"""
Visualize joint torques: measured, commanded, impedance, and coriolis+gravity.
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
    fig, axes = plt.subplots(7, 1, figsize=(14, 16), sharex=True)
    fig.suptitle('Joint Torques Comparison', fontsize=14, fontweight='bold')
    
    colors = {
        'measured': '#e74c3c',      # Red - actual measured torque
        'commanded': '#2ecc71',     # Green - commanded torque
        'impedance': '#3498db',     # Blue - impedance torque
        'coriolis_gravity': '#9b59b6'  # Purple - coriolis + gravity
    }
    
    for i in range(7):
        ax = axes[i]
        
        tau_measured = df[f'tau_J_{i+1}'].values
        tau_cmd = df[f'tau_cmd_{i+1}'].values
        tau_impedance = df[f'tau_impedance_{i+1}'].values
        tau_coriolis = df[f'tau_coriolis_{i+1}'].values
        tau_gravity = df[f'tau_gravity_{i+1}'].values
        tau_coriolis_gravity = tau_coriolis + tau_gravity
        
        ax.plot(time, tau_measured, color=colors['measured'], linewidth=1.2, 
                label='Measured (tau_J)', alpha=0.9)
        ax.plot(time, tau_cmd, color=colors['commanded'], linewidth=1.2, 
                label='Commanded (tau_cmd)', alpha=0.9)
        ax.plot(time, tau_impedance, color=colors['impedance'], linewidth=1.2, 
                label='Impedance', alpha=0.9)
        ax.plot(time, tau_coriolis_gravity, color=colors['coriolis_gravity'], linewidth=1.2, 
                label='Coriolis + Gravity', alpha=0.9)
        
        ax.set_ylabel(f'Joint {i+1}\n(Nm)', fontsize=10)
        ax.grid(True, alpha=0.3)
        
        if i == 0:
            ax.legend(loc='upper right', fontsize=8, ncol=2)
    
    axes[-1].set_xlabel('Time (s)', fontsize=12)
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(script_dir, 'visualize_torques.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Figure saved to: {output_path}")
    
    plt.show()

if __name__ == '__main__':
    main()
