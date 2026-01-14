#!/usr/bin/env python3
"""
Visualize joint torques: measured, commanded, impedance, and feedforward.
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
    fig, axes = plt.subplots(2, 4, figsize=(18, 8), sharex=True)
    fig.suptitle('Joint Torques Decomposition\n(tau_cmd = tau_feedforward + tau_impedance = (M*ddq + C + g) + tau_impedance)', 
                 fontsize=14, fontweight='bold')
    
    colors = {
        'commanded': '#2ecc71',     # Green - commanded torque
        'measured': '#e74c3c',      # Red - measured torque
        'gravity': '#9b59b6',       # Purple - gravity torque
        'inertia_coriolis': '#f39c12',  # Orange - inertia + coriolis
        'feedforward': '#e91e63',   # Pink - feedforward (M*ddq + C + g)
        'impedance': '#3498db'      # Blue - impedance torque
    }
    
    # Flatten axes array for easier indexing
    axes_flat = axes.flatten()
    
    for i in range(7):
        ax = axes_flat[i]
        
        tau_cmd = df[f'tau_cmd_{i+1}'].values
        tau_measured = df[f'tau_J_{i+1}'].values
        tau_gravity = df[f'tau_gravity_{i+1}'].values
        tau_inertia = df[f'tau_inertia_{i+1}'].values
        tau_coriolis = df[f'tau_coriolis_{i+1}'].values
        tau_impedance = df[f'tau_impedance_{i+1}'].values
        
        # Calculate combined torques
        tau_inertia_coriolis = tau_inertia + tau_coriolis  # M*ddq + C
        tau_feedforward = tau_inertia + tau_coriolis + tau_gravity  # M*ddq + C + g
        
        # Plot in order of importance (back to front)
        ax.plot(time, tau_feedforward, color=colors['feedforward'], linewidth=1.2, 
                label='Feedforward (M*ddq+C+g)', alpha=0.7, linestyle='--')
        ax.plot(time, tau_inertia_coriolis, color=colors['inertia_coriolis'], linewidth=1.2, 
                label='Inertia+Coriolis (M*ddq+C)', alpha=0.7, linestyle=':')
        ax.plot(time, tau_gravity, color=colors['gravity'], linewidth=1.2, 
                label='Gravity', alpha=0.7, linestyle=':')
        ax.plot(time, tau_impedance, color=colors['impedance'], linewidth=1.3, 
                label='Impedance', alpha=0.8)
        ax.plot(time, tau_measured, color=colors['measured'], linewidth=1.5, 
                label='Measured', alpha=0.9)
        ax.plot(time, tau_cmd, color=colors['commanded'], linewidth=1.5, 
                label='Commanded', alpha=0.9)
        
        ax.set_ylabel(f'Joint {i+1} (Nm)', fontsize=9)
        ax.grid(True, alpha=0.3)
        
        if i == 0:
            ax.legend(loc='best', fontsize=7, ncol=2)
        
        # Add x-label to bottom row
        if i >= 4:  # Bottom row (indices 4, 5, 6, 7)
            ax.set_xlabel('Time (s)', fontsize=10)
    
    # Hide the last subplot (index 7) as we only have 7 joints
    axes_flat[7].axis('off')
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(script_dir, 'visualize_torques.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Figure saved to: {output_path}")
    
    plt.show()

if __name__ == '__main__':
    main()
