#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

def plot_vc_path(csv_file='vc_path.csv'):
    try:
        df = pd.read_csv(csv_file)
        
        x = df['x'].values
        y = df['y'].values
        
        plt.figure(figsize=(12, 10))
        
        plt.plot(x, y, 'b-', linewidth=2, label='VC Path', alpha=0.6)
        
        plt.scatter(0, 0, c='green', s=300, marker='s', 
                   label='Charging Station (CS)', zorder=5, edgecolors='black', linewidths=2)
        
        plt.scatter(x[-1], y[-1], c='red', s=300, marker='o', 
                   label='Final VC Position', zorder=5, edgecolors='black', linewidths=2)
        
        plt.scatter(x[0], y[0], c='blue', s=150, marker='o', 
                   label='Start Position', zorder=4, edgecolors='black', linewidths=1.5)
        
        distance = np.sqrt(x[-1]**2 + y[-1]**2)
        
        plt.annotate(f'CS\n(0.0, 0.0)', 
                    xy=(0, 0), xytext=(0.3, 0.3),
                    fontsize=10, ha='left',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgreen', alpha=0.7))
        
        plt.annotate(f'Final Position\n({x[-1]:.3f}, {y[-1]:.3f})', 
                    xy=(x[-1], y[-1]), xytext=(x[-1]+0.3, y[-1]+0.3),
                    fontsize=10, ha='left',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='lightcoral', alpha=0.7))
        
        plt.title(f'Vacuum Cleaner Path Relative to Charging Station\n' + 
                 f'Final Distance from CS: {distance:.3f} meters', fontsize=16, fontweight='bold')
        
        plt.xlabel('X (meters)', fontsize=14)
        plt.ylabel('Y (meters)', fontsize=14)
        plt.grid(True, alpha=0.3, linestyle='--')
        plt.legend(fontsize=12, loc='best')
        plt.axis('equal')
        
        plt.tight_layout()
        plt.savefig('vc_path_plot.png', dpi=300, bbox_inches='tight')
        print(f"Plot saved as 'vc_path_plot.png'")
        
        plt.show()
        
        print(f"\n{'='*60}")
        print(f"Statistics:")
        print(f"{'='*60}")
        print(f"Total positions recorded: {len(x)}")
        print(f"Start position: ({x[0]:.3f}, {y[0]:.3f})")
        print(f"Final position: ({x[-1]:.3f}, {y[-1]:.3f})")
        print(f"Distance from CS: {distance:.3f} meters")
        
        path_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
        print(f"Total path length: {path_length:.3f} meters")
        
        print(f"{'='*60}")
        
    except FileNotFoundError:
        print(f"Error: Could not find file '{csv_file}'")
        print("Make sure the vacuum_tracker node has run and generated the CSV file.")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    if len(sys.argv) > 1:
        plot_vc_path(sys.argv[1])
    else:
        plot_vc_path()
