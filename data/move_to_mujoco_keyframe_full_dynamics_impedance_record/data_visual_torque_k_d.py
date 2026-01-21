#!/usr/bin/env python3
"""
Visualize joint impedance torques: tau_k (stiffness) and tau_d (damping).
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
    fig.suptitle('Impedance Torques: Stiffness (K) vs Damping (D)\n(tau_impedance = tau_k + tau_d)', 
                 fontsize=14, fontweight='bold')
    
    colors = {
        'tau_k': '#e74c3c',   # Red - stiffness torque
        'tau_d': '#3498db'    # Blue - damping torque
    }
    
    # Flatten axes array for easier indexing
    axes_flat = axes.flatten()
    
    for i in range(7):
        ax = axes_flat[i]
        
        tau_k = df[f'tau_k_{i+1}'].values
        tau_d = df[f'tau_d_{i+1}'].values
        
        ax.plot(time, tau_k, color=colors['tau_k'], linewidth=1.5, 
                label='tau_k (K×pos_err)', alpha=0.9)
        ax.plot(time, tau_d, color=colors['tau_d'], linewidth=1.5, 
                label='tau_d (D×vel_err)', alpha=0.9)
        
        ax.set_ylabel(f'Joint {i+1} (Nm)', fontsize=9)
        ax.grid(True, alpha=0.3)
        
        # Add x-label to bottom row
        if i >= 4:  # Bottom row (indices 4, 5, 6, 7)
            ax.set_xlabel('Time (s)', fontsize=10)
    
    # Use the 8th subplot for legend
    ax_legend = axes_flat[7]
    ax_legend.axis('off')
    
    # Create dummy lines for legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color=colors['tau_k'], linewidth=2, label='tau_k (Stiffness: K × position error)'),
        Line2D([0], [0], color=colors['tau_d'], linewidth=2, label='tau_d (Damping: D × velocity error)'),
    ]
    ax_legend.legend(handles=legend_elements, loc='center', fontsize=11, frameon=True, 
                     fancybox=True, shadow=True, ncol=1)
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(script_dir, 'visualize_torques_k_d.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Figure saved to: {output_path}")
    
    plt.show()

if __name__ == '__main__':
    main()
