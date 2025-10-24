import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_sensor_data_from_csvs(file_paths_dict):
    """
    Plots data from multiple CSV files, each in its own subplot of a 2x2 grid.

    Args:
        file_paths_dict (dict): A dictionary where keys are plot titles (usually filenames)
                                and values are the paths to the CSV files.
    """
    # Define consistent colors for the 6 data columns
    # Using Matplotlib's default 'tab10' colors for good visibility
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b'] 
    
    num_files_to_plot = len(file_paths_dict)
    if num_files_to_plot == 0:
        print("No files provided to plot.")
        return

    # Ensure we have at most 4 files for a 2x2 grid
    # If more are provided, only the first 4 are taken.
    files_to_process = list(file_paths_dict.items())
    if num_files_to_plot > 4:
        print("Warning: More than 4 files provided. Only the first 4 will be plotted in a 2x2 grid.")
        files_to_process = files_to_process[:4]
    
    nrows, ncols = 2, 2
    fig, axes = plt.subplots(nrows, ncols, figsize=(17, 12)) 
    axes = axes.flatten() # Flatten to easily iterate over axes, even if nrows=1 or ncols=1

    plot_successful_for_legend = False # Flag to check if any data was plotted

    for i, (plot_title, filepath) in enumerate(files_to_process):
        ax = axes[i]
        try:
            # Read CSV. Pandas will infer the number of columns.
            raw_df = pd.read_csv(filepath, header=None, engine='python')

            processed_df = pd.DataFrame() # Initialize as empty

            # We expect 6 data columns.
            # If the file has fewer than 6 columns in its widest row, it cannot contain valid data.
            if raw_df.shape[1] < 6:
                message = f"Widest row has {raw_df.shape[1]} columns (need 6)"
                ax.text(0.5, 0.5, message, ha='center', va='center', transform=ax.transAxes, fontsize=10, wrap=True)
                print(f"Info: File '{filepath}' - {message}. No valid data rows.")
            else:
                # Select the first 6 columns
                current_processed_df = raw_df.iloc[:, :6].copy()
                # Assign standard column names "0" through "5"
                current_processed_df.columns = [str(j) for j in range(6)]

                # Convert all 6 selected columns to numeric type.
                # Errors during conversion will be set to NaN.
                for col_name in current_processed_df.columns:
                    current_processed_df[col_name] = pd.to_numeric(current_processed_df[col_name], errors='coerce')
                
                # Drop rows that do not have a numeric value in ALL 6 chosen columns
                current_processed_df.dropna(inplace=True)
                processed_df = current_processed_df
            
            if processed_df.empty:
                # This message is shown if processing was attempted but yielded no valid rows
                if raw_df.shape[1] >= 6: 
                    ax.text(0.5, 0.5, "No valid 6-column numeric data rows", ha='center', va='center', transform=ax.transAxes, fontsize=10)
                    print(f"Info: No valid 6-column numeric data rows found in '{filepath}' after cleaning.")
            else:
                # Reset index to get sequential row numbers (0, 1, 2...) for the x-axis
                processed_df.reset_index(drop=True, inplace=True)
                
                # Create the time axis: row number / 10 Hz
                time_in_seconds = processed_df.index / 10.0

                # Plot each of the 6 columns
                for j, col_name in enumerate(processed_df.columns): # Columns are named '0' through '5'
                    ax.plot(time_in_seconds, processed_df[col_name], color=colors[j])
                
                plot_successful_for_legend = True # Mark that at least one subplot has data

        except pd.errors.EmptyDataError:
            ax.text(0.5, 0.5, "Empty file or unreadable", ha='center', va='center', transform=ax.transAxes, fontsize=10)
            print(f"Warning: File '{filepath}' is empty or could not be parsed.")
        except Exception as e:
            error_message = f"Error processing file:\n{str(e)[:100]}" # Truncate long errors
            ax.text(0.5, 0.5, error_message, ha='center', va='center', transform=ax.transAxes, wrap=True, fontsize=9)
            print(f"Error processing file '{filepath}': {e}")

        ax.set_title(plot_title, fontsize=12)
        ax.set_xlabel("Time (s)", fontsize=10)
        ax.set_ylabel("Value (bits)", fontsize=10)
        ax.set_ylim(0, 1023)
        ax.grid(True, linestyle='--', alpha=0.7)

    # If there are more axes than files plotted, hide the unused ones
    for j in range(len(files_to_process), nrows * ncols):
        fig.delaxes(axes[j])

    # Add a figure title
    fig.suptitle("OpenCR Sensor Data Comparison", fontsize=16)

    # Create a single shared legend for the entire figure
    if plot_successful_for_legend:
        legend_handles = [plt.Line2D([0], [0], color=colors[k], lw=2, label=str(k)) for k in range(6)]
        # Place legend below the subplots. `bbox_to_anchor` positions it relative to the figure.
        fig.legend(handles=legend_handles, loc='lower center', bbox_to_anchor=(0.5, 0.01), ncol=6, frameon=False, fontsize=10)
        # Adjust layout to prevent overlap.
        # rect=[left, bottom, right, top] in normalized figure coordinates.
        fig.tight_layout(rect=[0.03, 0.06, 0.97, 0.94]) # bottom: 0.06 for legend, top: 0.94 for suptitle
    else:
        # No legend, so less bottom padding needed
        fig.tight_layout(rect=[0.03, 0.02, 0.97, 0.94])
    
    # For saving the figure instead of showing:
    # plt.show()
    plt.savefig("sensor_plots.png", dpi=300)

# This part allows the script to be run directly.
# The CSV files are assumed to be in the same directory as this script,
# or you can provide absolute paths.
if __name__ == "__main__":
    # Define the files to plot. The keys will be used as plot titles.
    # Assumes the CSV files are in the same directory as this script.
    # If not, provide the full path to each file.
    # For example: 'C:/Users/YourUser/Documents/data/new_opencr_sensor1.csv' on Windows
    # or '/home/youruser/data/new_opencr_sensor1.csv' on Linux/macOS.

    # Get the directory where the script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    files_to_plot = {
        "new_opencr_sensor1.csv": os.path.join(script_dir, "new_opencr_sensor1.csv"),
        "new_opencr_sensor2.csv": os.path.join(script_dir, "new_opencr_sensor2.csv"),
        "old_opencr_sensor1.csv": os.path.join(script_dir, "old_opencr_sensor1.csv"),
        "old_opencr_sensor2.csv": os.path.join(script_dir, "old_opencr_sensor2.csv")
    }

    # Check if files exist before attempting to plot
    existing_files = {}
    for title, path in files_to_plot.items():
        if os.path.exists(path):
            existing_files[title] = path
        else:
            print(f"Warning: File not found at '{path}'. It will be skipped.")
    
    if not existing_files:
        print("Error: None of the specified CSV files were found. Please check the file paths.")
    else:
        plot_sensor_data_from_csvs(existing_files)