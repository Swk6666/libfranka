import numpy as np
import pandas as pd
import os

def main():
    # Load data from same directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(script_dir, 'trajectory_data.csv')
    
    print(f"Loading data from: {csv_path}")
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} data points")
    print()
    
    # Print first 5 time steps torque data
    for idx in range(5):
        time = df['time'].iloc[idx]
        print(f"=== Time step {idx}: t = {time:.6f} s ===")
        
        # tau_cmd
        tau_cmd = [df[f'tau_cmd_{j+1}'].iloc[idx] for j in range(7)]
        print(f"tau_cmd:        [{', '.join([f'{t:8.4f}' for t in tau_cmd])}]")
        
        # tau_feedforward
        tau_ff = [df[f'tau_feedforward_{j+1}'].iloc[idx] for j in range(7)]
        print(f"tau_feedforward:[{', '.join([f'{t:8.4f}' for t in tau_ff])}]")
        
        # tau_impedance
        tau_imp = [df[f'tau_impedance_{j+1}'].iloc[idx] for j in range(7)]
        print(f"tau_impedance:  [{', '.join([f'{t:8.4f}' for t in tau_imp])}]")
        
        # tau_J (measured)
        tau_J = [df[f'tau_J_{j+1}'].iloc[idx] for j in range(7)]
        print(f"tau_J (meas):   [{', '.join([f'{t:8.4f}' for t in tau_J])}]")
        
        # Verify: tau_cmd should equal tau_ff + tau_impedance
        tau_sum = [tau_ff[j] + tau_imp[j] for j in range(7)]
        print(f"tau_ff+tau_imp: [{', '.join([f'{t:8.4f}' for t in tau_sum])}]")
        print()

if __name__ == '__main__':
    main()