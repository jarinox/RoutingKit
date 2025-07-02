import sys
import folium

lines = [line.strip() for line in sys.stdin if line.strip()]

edges = []
lats, lons = [], []

for line in lines:
    parts = line.split()
    if len(parts) < 5:
        continue  # skip malformed lines
    from_lat, from_lon, to_lat, to_lon = map(float, parts[:4])
    is_shortcut = parts[4]
    labels = parts[5:] if len(parts) > 5 else []
    edges.append({
        "from": (from_lat, from_lon),
        "to": (to_lat, to_lon),
        "is_shortcut": is_shortcut,
        "labels": labels
    })
    lats.extend([from_lat, to_lat])
    lons.extend([from_lon, to_lon])

center_lat = sum(lats) / len(lats) if lats else 0
center_lon = sum(lons) / len(lons) if lons else 0
m = folium.Map(location=[center_lat, center_lon], zoom_start=13)

for edge in edges:
    color = "red" if edge["is_shortcut"] == "S" else "blue"
    tooltip = f"{'Shortcut' if edge['is_shortcut']=='S' else 'Original'}"
    if edge["labels"]:
        tooltip += " | " + ", ".join(edge["labels"])
    folium.PolyLine(
        [edge["from"], edge["to"]],
        color=color,
        weight=3,
        tooltip=tooltip
    ).add_to(m)

m.save("ch_visualization.html")
print("Visualization saved to ch_visualization.html")
