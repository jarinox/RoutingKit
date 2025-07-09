import sys
import folium
import pyproj
import math

lines = [line.strip() for line in sys.stdin if line.strip()]

nodes = []
edges = []
lats, lons = [], []

# Parse input sections
section = None
for line in lines:
    if line == "BEGIN NODES":
        section = "nodes"
        continue
    elif line == "BEGIN EDGES":
        section = "edges"
        continue
    if section == "nodes":
        parts = line.split()
        if len(parts) < 3:
            continue
        lat, lon, rank = float(parts[0]), float(parts[1]), parts[2]
        nodes.append({"lat": lat, "lon": lon, "rank": rank})
        lats.append(lat)
        lons.append(lon)
    elif section == "edges":
        parts = line.split()
        if len(parts) < 6:
            continue  # skip malformed lines
        from_lat, from_lon, to_lat, to_lon = map(float, parts[:4])
        is_shortcut = parts[4]
        is_upward = parts[5] == "F"
        ranks = parts[6]
        weight = parts[7]
        labels = parts[8:] if len(parts) > 7 else []
        edges.append({
            "from": (from_lat, from_lon),
            "to": (to_lat, to_lon),
            "is_shortcut": is_shortcut,
            "is_upward": is_upward,
            "labels": labels,
            "ranks": ranks,
            "weight": weight
        })
        lats.extend([from_lat, to_lat])
        lons.extend([from_lon, to_lon])

center_lat = sum(lats) / len(lats) if lats else 0
center_lon = sum(lons) / len(lons) if lons else 0
m = folium.Map(location=[center_lat, center_lon], zoom_start=13)

# Draw nodes
for node in nodes:
    folium.CircleMarker(
        location=(node["lat"], node["lon"]),
        radius=3,
        color="#222",
        fill=True,
        fill_color="#fff",
        fill_opacity=0.8,
        tooltip=f"Rank: {node['rank']}"
    ).add_to(m)

def bearing(from_lat, from_lon, to_lat, to_lon):
    geod = pyproj.Geod(ellps="WGS84")
    return geod.inv(from_lon, from_lat, to_lon, to_lat)[0] + 270

# Draw edges
for edge in edges:
    if edge["is_shortcut"] == "S":
        color = "#ef0020" if edge["is_upward"] else "#ff8f45"
    else:
        color = "#0059ff" if edge["is_upward"] else "#5de2d9"

    tooltip = f"{'Shortcut' if edge['is_shortcut']=='S' else 'Original'}"
    tooltip += f" | {edge['ranks']} | {edge['weight']}"
    if edge["labels"]:
        tooltip += " | " + ", ".join(edge["labels"])
    
    rot = bearing(
        edge["from"][0], edge["from"][1],
        edge["to"][0], edge["to"][1]
    )
    
    dx = edge["to"][1] - edge["from"][1]
    dy = edge["to"][0] - edge["from"][0]
    length = math.hypot(dx, dy)
    if length == 0:
        offset_dx, offset_dy = 0, 0
    else:
        perp_left = (-dy / length, dx / length)
        perp_right = (dy / length, -dx / length)
        offset_amount = 0.00001
        offset_amount *= (((rot % 360) / 360)+1)
        if edge["is_upward"]:
            offset = perp_left
        else:
            offset = perp_right
        offset_dx = offset[0] * offset_amount
        offset_dy = offset[1] * offset_amount
        edge["from"] = (edge["from"][0] + offset_dx, edge["from"][1] + offset_dy)
        edge["to"] = (edge["to"][0] + offset_dx, edge["to"][1] + offset_dy)

    folium.PolyLine(
        [edge["from"], edge["to"]],
        color=color,
        weight=2,
        tooltip=tooltip
    ).add_to(m)

    center = (
        (edge["from"][0] + edge["to"][0]) / 2,
        (edge["from"][1] + edge["to"][1]) / 2
    )
    folium.RegularPolygonMarker(
        location=center,
        number_of_sides=3,
        radius=5,
        rotation=(rot + 180 if edge["is_upward"] else rot),
        color=color,
        fill=True,
        fill_color=color,
        tooltip=tooltip
    ).add_to(m)

m.save("ch_visualization.html")
print("Visualization saved to ch_visualization.html")
