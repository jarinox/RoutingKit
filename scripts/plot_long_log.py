import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# This script requires pandas and matplotlib.
# You can install them with:
# pip install pandas matplotlib

def parse_log_file(file_path):
    """
    Parses the log file to extract performance data for each run.

    Args:
        file_path (str): The path to the log file.

    Returns:
        pd.DataFrame: A DataFrame containing the extracted data, with each row representing a run.
    """
    with open(file_path, 'r') as f:
        content = f.read()

    runs = content.strip().split('===== Run')
    data = []

    for run_str in runs:
        if not run_str.strip():
            continue

        run_data = {}

        # Extract nodes and build time
        build_match = re.search(r'(\d+) nodes in (\d+) microseconds', run_str)
        if build_match:
            run_data['nodes'] = int(build_match.group(1))
            run_data['build_time'] = int(build_match.group(2))

        # Extract edges and shortcuts
        edges_match = re.search(r'(\d+) edges and (\d+) shortcuts', run_str)
        if edges_match:
            run_data['edges'] = int(edges_match.group(1))
            run_data['shortcuts'] = int(edges_match.group(2))

        # Extract mode times
        mode_matches = re.findall(r'Mode (\d+): \d+ changes in (\d+) microseconds', run_str)
        for mode, time in mode_matches:
            run_data[f'mode_{mode}'] = int(time)
        
        if build_match: # Only add if we have the basic run info
            data.append(run_data)

    return pd.DataFrame(data)

def process_data(df):
    """
    Processes the extracted data to calculate cumulative mode times and average data points.

    Args:
        df (pd.DataFrame): The DataFrame with raw data from the log file.

    Returns:
        pd.DataFrame: A DataFrame with processed and averaged data, ready for plotting.
    """
    if df.empty:
        return pd.DataFrame()

    # Calculate cumulative mode times (from mode 1 upwards)
    mode_cols = [f'mode_{i}' for i in range(1, 6) if f'mode_{i}' in df.columns]
    
    # Ensure we only work with existing mode columns
    if mode_cols:
        # Convert relevant columns to numeric, coercing errors
        for col in mode_cols:
            df[col] = pd.to_numeric(df[col], errors='coerce')
        
        # Drop rows with NaN values that might result from coercion
        df = df.dropna(subset=mode_cols)

        # Now perform the cumulative sum
        df[mode_cols] = df[mode_cols].cumsum(axis=1)


    # Average every 10 rows
    group_size = 20
    # Create a group index
    df['group'] = np.floor(df.index / group_size)
    
    # Group by the new index and calculate the mean
    averaged_df = df.groupby('group').mean()

    return averaged_df

def plot_data(df):
    """
    Plots the processed data.

    Args:
        df (pd.DataFrame): The DataFrame with averaged data.
    """
    if df.empty:
        print("DataFrame is empty, nothing to plot.")
        return

    plt.style.use('ggplot')
    fig, ax = plt.subplots(figsize=(12, 8))

    ax.plot(df['nodes'], df['build_time'], label='Build Time', marker='o')

    mode_titles = [
        "single change",
        "1% changed",
        "5% changed",
        "10% changed",
        "20% changed",
        "50% changed"
    ]

    for i in range(2):
        mode_col = f'mode_{i}'
        if mode_col in df.columns:
            ax.plot(df['nodes'], df[mode_col], label=mode_titles[i], marker='o')

    ax.set_xlabel('Number of Nodes (averaged over 10 runs)')
    ax.set_ylabel('Time (seconds)')
    ax.set_title('Performance Analysis')
    ax.legend()
    ax.grid(True)

    #ax.set_yscale('log')
    
    plt.tight_layout()
    plt.savefig('build_performance.png')
    plt.show()

if __name__ == "__main__":
    # Path to the log file. Change this if your log file has a different name.
    log_file_path = 'ch_build_maintenance_b.txt'
    
    try:
        raw_data_df = parse_log_file(log_file_path)
        if not raw_data_df.empty:
            processed_df = process_data(raw_data_df.copy()) # Use copy to avoid SettingWithCopyWarning
            for col in ['build_time'] + [f'mode_{i}' for i in range(0, 6)]:
                if col in processed_df.columns:
                    processed_df[col] = processed_df[col] / 1_000_000  # Convert microseconds to seconds
            plot_data(processed_df)
        else:
            print(f"No data could be parsed from {log_file_path}.")
    except FileNotFoundError:
        print(f"Error: The file {log_file_path} was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

