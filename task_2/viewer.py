import json
import folium

# Read the JSON file
with open('/home/fsociety/Documents/Uni-Stuttgart/SS2024/OpenMaps_FachPrak/task_2/landmarks.json', 'r') as file:
    data = json.load(file)

print(len(data))
# Initialize a map centered on an average location
map_center = [20, 0]  # Slightly adjusted center for better visibility of points
m = folium.Map(location=map_center, zoom_start=2)

# Plot points and lines
for item in data:
    if len(item) == 2:
        # Plot point with a circle marker
        folium.CircleMarker(
            location=[item[0], item[1]],
            radius=8,  # Increase the radius to make points more visible
            color='red',
            fill=True,
            fill_color='red',
            fill_opacity=0.6
        ).add_to(m)
    elif len(item) == 4:
        # Plot line
        folium.PolyLine(
            locations=[[item[0], item[1]], [item[2], item[3]]],
            color='blue', weight=2.5, opacity=1
        ).add_to(m)

# Save the map to an HTML file
m.save('/home/fsociety/Documents/Uni-Stuttgart/SS2024/OpenMaps_FachPrak/task_2/map.html')
