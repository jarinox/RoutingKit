import sys
import folium
from folium import plugins

def visualize_path():
    path_segments = []
    start_point_overall = None
    end_point_overall = None
    
    lines = sys.stdin.readlines()

    if not lines:
        print("No input received. Please pipe data from your C++ program.", file=sys.stderr)
        sys.exit(1)

    try:
        header_parts = lines[0].strip().split()
        if len(header_parts) == 4:
            start_point_overall = (float(header_parts[0]), float(header_parts[1])) # (lat, lon)
            end_point_overall = (float(header_parts[2]), float(header_parts[3]))   # (lat, lon)
        else:
            print("Warning: First line does not contain 4 coordinates for overall start/end points. Skipping.", file=sys.stderr)

        # Each line: lat lon label
        current_segment_start = None
        if start_point_overall:
            current_segment_start = start_point_overall
        elif len(lines) > 1: # If no overall start, use the first path point as start
            first_path_line_parts = lines[1].strip().split(maxsplit=2) # maxsplit=2 to handle spaces in labels
            if len(first_path_line_parts) >= 2:
                current_segment_start = (float(first_path_line_parts[0]), float(first_path_line_parts[1]))
            else:
                 print("Error: First path line is malformed.", file=sys.stderr)
                 sys.exit(1)


        for line_num, line in enumerate(lines[1:]): # Skip the header line (start/end points)
            parts = line.strip().split(maxsplit=2) # Split at most twice: lat, lon, then rest is label
            
            if len(parts) >= 2:
                lat = float(parts[0])
                lon = float(parts[1])
                label = parts[2].strip().strip('"') if len(parts) > 2 else "" # Remove quotes from label if present
                
                if current_segment_start:
                    segment_end = (lat, lon)
                    path_segments.append({
                        "start": current_segment_start,
                        "end": segment_end,
                        "label": label
                    })
                    current_segment_start = segment_end # Next segment starts where this one ended
                else:
                    current_segment_start = (lat, lon)

                    if not path_segments and label:
                         path_segments.append({
                            "start": start_point_overall if start_point_overall else (lat,lon),
                            "end": (lat,lon),
                            "label": label
                         })
                         current_segment_start = (lat,lon)
                    elif not path_segments:
                        pass

            else:
                print(f"Warning: Skipping malformed line {line_num + 2}: '{line.strip()}'", file=sys.stderr)

    except ValueError as e:
        print(f"Error parsing numerical data from input: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"An unexpected error occurred: {e}", file=sys.stderr)
        sys.exit(1)

    if not path_segments and (not start_point_overall or not end_point_overall):
        print("No valid path segments or overall start/end points found in the input.", file=sys.stderr)
        sys.exit(0)

    # Reconstruct the full path for the main PolyLine visualization
    full_path_coords = []
    if start_point_overall:
        full_path_coords.append(list(start_point_overall)) # (lat, lon)
    
    for segment in path_segments:
        full_path_coords.append(list(segment["end"])) # Add end point of each segment

    if not full_path_coords:
        print("No coordinates to draw a path.", file=sys.stderr)
        sys.exit(0)

    center_lat, center_lon = full_path_coords[len(full_path_coords) // 2]
    m = folium.Map(location=[center_lat, center_lon], zoom_start=14)

    main_polyline = folium.PolyLine(
        locations=full_path_coords,
        color="blue",
        weight=5,
        opacity=0.7,
        tooltip="Shortest Path"
    ).add_to(m)

    # Add labels for each segment
    for i, segment in enumerate(path_segments):
        if segment["label"]:
            segment_coords = [list(segment["start"]), list(segment["end"])]
            
            segment_polyline = folium.PolyLine(
                locations=segment_coords,
                color="rgba(0,0,0,0)", # Transparent line
                weight=0,
                opacity=0
            ).add_to(m)

            plugins.PolyLineTextPath(
                segment_polyline,
                text=segment["label"],
                repeat=False,
                offset=7,
                attributes={'fill': 'black', 'font-weight': 'bold', 'font-size': '12'} # Customize text style
            ).add_to(m)


    # Add markers for start and end
    if start_point_overall:
        folium.Marker(
            location=start_point_overall,
            popup="Start",
            icon=folium.Icon(color='green')
        ).add_to(m)
    
    if end_point_overall:
        folium.Marker(
            location=end_point_overall,
            popup="End",
            icon=folium.Icon(color='red')
        ).add_to(m)

    folium.LayerControl().add_to(m)

    output_html_file = "shortest_path_map.html"
    m.save(output_html_file)
    print(f"Map saved to {output_html_file}")
    print("Open this file in your web browser to view the path with labels.")

if __name__ == "__main__":
    visualize_path()
