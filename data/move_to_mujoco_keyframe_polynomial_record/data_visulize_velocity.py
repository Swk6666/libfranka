#!/usr/bin/env python3
"""
Visualize trajectory data from Franka robot motion recording.
Plots joint velocities for all 7 joints.
"""

import os
import pandas as pd
import matplotlib.pyplot as plt

def main():
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(script_dir, "trajectory_data.csv")
    
    # Check if file exists
    if not os.path.exists(csv_path):
        print(f"Error: File not found: {csv_path}")
        return
    
    # Load data
    print(f"Loading data from: {csv_path}")
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} data points")
    
    # Create figure with 7 subplots (one for each joint)
    fig, axes = plt.subplots(4, 2, figsize=(14, 12))
    axes = axes.flatten()
    
    # Plot each joint velocity
    for j in range(7):
        ax = axes[j]
        
        # Plot actual joint velocity
        ax.plot(df['time'], df[f'dq_actual_{j+1}'], 'g-', 
                label='Actual Velocity', linewidth=1.0)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (rad/s)')
        ax.set_title(f'Joint {j+1} Velocity')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
    
    # Hide the 8th subplot (we only have 7 joints)
    axes[7].axis('off')
    
    # Add main title
    fig.suptitle('Franka Robot Joint Velocities', 
                 fontsize=14, fontweight='bold')
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(script_dir, "joint_velocities.png")
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Figure saved to: {output_path}")
    
    # Show plot
    plt.show()

if __name__ == "__main__":
    main()
