import sys
import folium

def visualize_graph():
    edges = []
    all_coordinates = []
    
    lines = sys.stdin.readlines()

    if not lines:
        print("No input received. Please pipe data from your C++ program.", file=sys.stderr)
        sys.exit(1)

    try:
        start_line = 0
        if lines and "Entire graph" in lines[0]:
            start_line = 1

        # Each line: lat_start lon_start lat_end lon_end
        for line_num, line in enumerate(lines[start_line:]):
            parts = line.strip().split()
            
            if len(parts) >= 4:
                lat_start = float(parts[0])
                lon_start = float(parts[1])
                lat_end = float(parts[2])
                lon_end = float(parts[3])
                
                start_point = (lat_start, lon_start)
                end_point = (lat_end, lon_end)
                
                edges.append({
                    "start": start_point,
                    "end": end_point
                })
                
                all_coordinates.extend([start_point, end_point])
                
            else:
                print(f"Warning: Skipping malformed line {line_num + start_line + 1}: '{line.strip()}'", file=sys.stderr)

    except ValueError as e:
        print(f"Error parsing numerical data from input: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"An unexpected error occurred: {e}", file=sys.stderr)
        sys.exit(1)

    if not edges:
        print("No valid edges found in the input.", file=sys.stderr)
        sys.exit(0)

    if all_coordinates:
        center_lat = sum(coord[0] for coord in all_coordinates) / len(all_coordinates)
        center_lon = sum(coord[1] for coord in all_coordinates) / len(all_coordinates)
    else:
        center_lat, center_lon = 0, 0

    m = folium.Map(location=[center_lat, center_lon], zoom_start=12)

    # Add all edges as individual polylines
    for edge in edges:
        edge_coords = [list(edge["start"]), list(edge["end"])]
        
        folium.PolyLine(
            locations=edge_coords,
            color="blue",
            weight=1,
            opacity=0.6
        ).add_to(m)

    folium.LayerControl().add_to(m)

    output_html_file = "graph_visualization.html"
    m.save(output_html_file)
    print(f"Graph visualization saved to {output_html_file}")
    print(f"Visualized {len(edges)} edges from the graph.")
    print("Open this file in your web browser to view the complete graph.")

if __name__ == "__main__":
    visualize_graph()
