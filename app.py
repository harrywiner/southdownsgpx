# Coordinate conversion
from pyproj import CRS, Transformer

# Data io
import pandas as pd
import gpxpy

# Data visualization
import folium
from geopy.distance import geodesic
import pandas as pd


## Conversion
# Define the coordinate reference systems
osgb36 = CRS("EPSG:27700")  # OSGB36, the British National Grid
wgs84 = CRS("EPSG:4326")    # WGS84, used for GPS coordinates
transformer = Transformer.from_crs(osgb36, wgs84, always_xy=True)

def convert_coordinates(easting, northing):
    # Convert the coordinates
    lon, lat = transformer.transform(easting, northing)
    return lon, lat

# Read GPX file
with open('sdw.gpx', 'r') as gpx_file:
    gpx = gpxpy.parse(gpx_file)

def find_closest_point(lon, lat):
    # Returns point object and index
    minDist = 100
    closestPoint = None
    closestIdx = None
    for idx, point in enumerate(gpx.tracks[0].segments[0].points):
        dist = ((point.latitude - lat)**2 + (point.longitude - lon)**2)**0.5
        if dist < minDist:
            minDist = dist
            closestPoint = point
            closestIdx = idx
    return closestPoint, closestIdx

def calculate_elevation_gain(startIdx, endIdx):
    total_uphill = 0
    route = gpx.tracks[0].segments[0].points
    for i in range(startIdx, endIdx):
        elevation_change = route[i+1].elevation - route[i].elevation
        if elevation_change > 0:
            total_uphill += elevation_change 
    return total_uphill

def calculate_total_distance(startIdx, endIdx):
    total_distance = 0
    route = gpx.tracks[0].segments[0].points
    for i in range(startIdx, endIdx):
        start_point = (route[i].latitude, route[i].longitude)
        end_point = (route[i+1].latitude, route[i+1].longitude)
        distance = geodesic(start_point, end_point).kilometers
        total_distance += distance
    return total_distance


def find_and_calculate_uphill(startLon, startLat, endLon, endLat ):
    # Searches the gpx for the closest index for each point
    # displays coordinates for both and calculates the total elevation increase

    start, startIdx = find_closest_point(startLon, startLat)
    end, endIdx = find_closest_point(endLon, endLat)
    print(f"Start: {start.latitude}, {start.longitude}")
    print(f"End: {end.latitude}, {end.longitude}")
    print(f"Elevation gain: {calculate_elevation_gain(startIdx, endIdx)}")
    print(f"Total distance: {calculate_total_distance(startIdx, endIdx)}")

def create_dataframe(coordinates):
    data = []
    for i in range(len(coordinates) - 1):
        startLon, startLat = coordinates[i][0], coordinates[i][1]
        endLon, endLat = coordinates[i+1][0], coordinates[i+1][1]
        start, startIdx = find_closest_point(startLon, startLat)
        end, endIdx = find_closest_point(endLon, endLat)
        elevation_gain = calculate_elevation_gain(startIdx, endIdx)
        total_distance = calculate_total_distance(startIdx, endIdx)
        data.append({
            'Start Latitude': start.latitude,
            'Start Longitude': start.longitude,
            'End Latitude': end.latitude,
            'End Longitude': end.longitude,
            'Elevation Gain': elevation_gain,
            'Total Distance': total_distance
        })
    df = pd.DataFrame(data)
    return df


## Day 1
beaconIdx = 341

startLon, startLat = gpx.tracks[0].segments[0].points[0].longitude, gpx.tracks[0].segments[0].points[0].latitude
endLon, endLat = gpx.tracks[0].segments[0].points[beaconIdx].longitude, gpx.tracks[0].segments[0].points[beaconIdx].latitude

# find_and_calculate_uphill(startLon, startLat, endLon, endLat)

### Day 2

campsite_coordinates = [[447850,129550 ],[460400, 122400], [471750, 118700], [486460, 116880], [502750, 111850]]

campsite_gps = [convert_coordinates(x, y) for x, y in campsite_coordinates]

print(campsite_gps)
for i in range(len(campsite_gps) - 1):
    print(f"Day {i+1}")
    find_and_calculate_uphill(campsite_gps[i][0], campsite_gps[i][1], campsite_gps[i+1][0], campsite_gps[i+1][1])

df = create_dataframe(campsite_gps)
print(df)
print(sum(df['Total Distance']))