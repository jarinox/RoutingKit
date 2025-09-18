import networkx as nx
import matplotlib.pyplot as plt
import re
import random

def visualize_graph(file_path):
    """
    Visualizes a graph from a file.

    The file format is expected to be:
    - First line: "N nodes" where N is the number of nodes.
    - N lines with node ranks: "Node i rank r"
    - A line "Edges:"
    - Subsequent lines: "u -> v (weight,mid_node)" for each edge.

    Nodes are displayed in a circular layout. Edges are colored based on whether
    they are shortcuts (mid_node != 4294967295).
    """
    try:
        with open(file_path, 'r') as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"Error: The file '{file_path}' was not found.")
        return

    num_nodes_match = re.match(r'(\d+) nodes', lines[0])
    if not num_nodes_match:
        print("Error: Could not parse the number of nodes from the first line.")
        return
    
    num_nodes = int(num_nodes_match.group(1))

    G = nx.MultiDiGraph()

    edge_lines_start = -1
    for i, line in enumerate(lines):
        if "Edges:" in line:
            edge_lines_start = i + 1
            break
        
        node_match = re.match(r'Node (\d+) rank (\d+)', line)
        if node_match:
            node_id, rank = map(int, node_match.groups())
            if not G.has_node(node_id):
                G.add_node(node_id, rank=rank)

    if not G.nodes():
        G.add_nodes_from(range(num_nodes))

    if edge_lines_start == -1:
        print("Warning: 'Edges:' separator not found in file. Assuming no edges.")
        edge_lines_start = len(lines)

    for line in lines[edge_lines_start:]:
        line = line.strip()
        if not line:
            continue
        match = re.match(r'(\d+)\s*->\s*(\d+)\s*\((\d+),(\d+)\)', line)
        if match:
            u, v, weight, mid_node = map(int, match.groups())
            
            edge_type = 'normal'
            if weight == 2147483647:
                edge_type = 'inf_weight'
            elif mid_node != 4294967295:
                edge_type = 'shortcut'
            
            G.add_edge(u, v, weight=weight, mid_node=mid_node, type=edge_type)
        else:
            print(f"Warning: Could not parse edge from line: '{line}'")

    pos = nx.circular_layout(G)
    
    plt.figure(figsize=(14, 14))

    # Draw nodes
    nx.draw_networkx_nodes(G, pos, node_color='skyblue', node_size=500, alpha=0.9)
    
    # Draw node labels
    labels = {node: f"{node} ({data.get('rank', '')})" for node, data in G.nodes(data=True)}
    nx.draw_networkx_labels(G, pos, labels=labels, font_size=10, font_family='sans-serif')

    # Draw edges and their labels
    for u, v, key, data in G.edges(keys=True, data=True):
        rad = random.uniform(-0.3, 0.3)
        connectionstyle = f'arc3,rad={rad}'
        edge_type = data.get('type', 'normal')
        weight = data.get('weight')

        if edge_type == 'normal':
            nx.draw_networkx_edges(G, pos, edgelist=[(u, v, key)], connectionstyle=connectionstyle, edge_color='black', width=1.0, alpha=0.6, arrows=True, arrowsize=30)
            if weight is not None and weight != 2147483647:
                nx.draw_networkx_edge_labels(G, pos, edge_labels={(u, v): weight}, font_color='darkgreen', font_size=8, label_pos=0.3, bbox=dict(facecolor='white', alpha=0, edgecolor='none'), connectionstyle=connectionstyle)
        elif edge_type == 'shortcut':
            nx.draw_networkx_edges(G, pos, edgelist=[(u, v, key)], connectionstyle=connectionstyle, edge_color='red', width=1.5, style='dashed', alpha=0.8, arrows=True, arrowsize=30)
            if weight is not None and weight != 2147483647:
                nx.draw_networkx_edge_labels(G, pos, edge_labels={(u, v): weight}, font_color='darkred', font_size=8, label_pos=0.3, bbox=dict(facecolor='white', alpha=0, edgecolor='none'), connectionstyle=connectionstyle)
        elif edge_type == 'inf_weight':
            nx.draw_networkx_edges(G, pos, edgelist=[(u, v, key)], connectionstyle=connectionstyle, edge_color='gray', width=0.5, alpha=0.2, arrows=False)

    # Create a legend
    legend_elements = [
        plt.Line2D([0], [0], color='black', lw=1, label='Normal Edge'),
        plt.Line2D([0], [0], color='red', lw=1.5, linestyle='--', label='Shortcut Edge'),
        plt.Line2D([0], [0], color='gray', lw=0.5, label='Infinite Weight Edge')
    ]
    plt.legend(handles=legend_elements, loc='upper right')

    plt.title("Contraction Hierarchy Visualization", size=15)
    plt.axis('off')
    
    if 'before' in file_path:
        output_filename = "generated/before.png"
    else:
        output_filename = "generated/after.png"

    plt.savefig(output_filename, format='png', dpi=300)
    print(f"Graph visualization saved to '{output_filename}'")
    # To display the plot in a window, uncomment the following line
    # plt.show()

if __name__ == '__main__':
    # Assuming the script is in the 'scripts' directory and the data file is in the parent directory
    visualize_graph('generated/debug_graph_before.txt')
    visualize_graph('generated/debug_graph_after.txt')
