import re
import pandas as pd
import matplotlib.pyplot as plt

def parse_log_file(file_path):
    """
    Parses the log file to extract nodes, microseconds, edges, and shortcuts.
    """
    data = []
    with open(file_path, 'r') as f:
        lines = f.readlines()
        i = 0
        while i < len(lines) - 1:
            # Extract nodes and microseconds
            match1 = re.match(r'(\d+) nodes in (\d+) microseconds', lines[i])
            if match1:
                nodes = int(match1.group(1))
                microseconds = int(match1.group(2))
                
                # Extract edges and shortcuts from the next line
                match2 = re.match(r'(\d+) edges and (\d+) shortcuts', lines[i+1])
                if match2:
                    edges = int(match2.group(1))
                    shortcuts = int(match2.group(2))
                    
                    data.append({
                        'nodes': nodes,
                        'microseconds': microseconds,
                        'edges': edges,
                        'shortcuts': shortcuts
                    })
                i += 2
            else:
                i += 1
    return pd.DataFrame(data)

def compute_averages(df, window_size=20):
    """
    Computes the rolling average for the data.
    """
    return df.rolling(window=window_size).mean()

def plot_data(df, averaged_df):
    """
    Plots the original and averaged data.
    """
    fig, axes = plt.subplots(3, 1, figsize=(12, 18), sharex=True)
    
    # Plot Seconds
    axes[0].plot(df['nodes'], df['seconds'], label='Original Seconds', alpha=0.5)
    axes[0].plot(averaged_df['nodes'], averaged_df['seconds'], label='Averaged Seconds (20-pocket)')
    axes[0].set_ylabel('Seconds')
    axes[0].set_title('Build Performance Analysis')
    axes[0].legend()
    axes[0].grid(True)

    # Plot Edges
    axes[1].plot(df['nodes'], df['edges'], label='Original Edges', alpha=0.5)
    axes[1].plot(averaged_df['nodes'], averaged_df['edges'], label='Averaged Edges (20-pocket)')
    axes[1].set_ylabel('Edges')
    axes[1].legend()
    axes[1].grid(True)

    # Plot Shortcuts
    axes[2].plot(df['nodes'], df['shortcuts'], label='Original Shortcuts', alpha=0.5)
    axes[2].plot(averaged_df['nodes'], averaged_df['shortcuts'], label='Averaged Shortcuts (20-pocket)')
    axes[2].set_xlabel('Number of Nodes')
    axes[2].set_ylabel('Shortcuts')
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig('build_performance.png')
    plt.show()

if __name__ == "__main__":
    log_file = 'ch_build_with_add_plus.txt'
    
    # 1. Extract data
    data_df = parse_log_file(log_file)
    
    if not data_df.empty:
        data_df['seconds'] = data_df['microseconds'] / 1_000_000
        # 2. Compute averages
        averaged_df = compute_averages(data_df)
        
        # 3. Plot data
        plot_data(data_df, averaged_df)
        print("Plot saved as build_performance.png")
    else:
        print("No data extracted from the log file.")

