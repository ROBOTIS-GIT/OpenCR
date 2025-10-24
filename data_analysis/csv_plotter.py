import pandas as pd
import matplotlib.pyplot as plt
import sys

def plot_csv_data(filename):
    try:
        # Read CSV file without headers
        df = pd.read_csv(filename, header=None)
        print(f"Loaded data with {len(df)} samples and {len(df.columns)} columns")
        
        # Create the plot
        plt.figure(figsize=(12, 8))
        
        # Default series names (can be customized here)
        series_names = ['s1', 's2', 's3']  # ['u1', 'u2', 's1', 's2', 's3', 'u3']
        
        # Color mapping for specific series
        color_map = {
            's1': 'blue',
            's2': 'red', 
            's3': 'green'
        }
        default_color = 'lightgray'
        
        # Plot each column and calculate standard deviation
        print("\nStandard Deviations:")
        print("-" * 30)
        
        # Store stats for s1, s2, s3 for the text box
        sensor_stats = {}
        
        for i, col in enumerate(df.columns):
            # Use default name if we have enough, otherwise use column index
            name = series_names[i] if i < len(series_names) else f'col{i}'
            
            # Get color for this series
            color = color_map.get(name, default_color)
            
            # Plot the data
            plt.plot(df.index, df[col], label=name, linewidth=1.5, color=color)
            
            # Calculate and print standard deviation
            std_dev = df[col].std()
            mean_val = df[col].mean()
            print(f"{name}: σ = {std_dev:.2f}, μ = {mean_val:.2f}")
            
            # Store stats for sensor series
            if name in ['s1', 's2', 's3']:
                sensor_stats[name] = {'std': std_dev, 'mean': mean_val}
        
        # Customize the plot
        plt.xlabel('Sample Number')
        plt.ylabel('ADC Value (12-bit)')
        plt.title('ADC Data Collection')
        plt.ylim(0, 4100)
        plt.xlim(0,len(df))
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Add statistics text box for sensor series
        if sensor_stats:
            stats_text = "Sensor Statistics (σ):\n"
            for sensor in ['s1', 's2', 's3']:
                if sensor in sensor_stats:
                    std_val = sensor_stats[sensor]['std']
                    stats_text += f"{sensor}: {std_val:.1f}\n"
            
            # Add text box in upper right corner
            plt.text(0.98, 0.98, stats_text.strip(), 
                    transform=plt.gca().transAxes,
                    fontsize=10,
                    verticalalignment='top',
                    horizontalalignment='right',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.8, edgecolor='gray'))
        
        # Show the plot
        plt.tight_layout()
        plt.show()
        
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found!")
    except pd.errors.EmptyDataError:
        print(f"Error: File '{filename}' is empty!")
    except Exception as e:
        print(f"Error reading file: {e}")

if __name__ == "__main__":
    # Get filename from command line argument or prompt user
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = input("Enter CSV filename: ")
    
    plot_csv_data(filename)