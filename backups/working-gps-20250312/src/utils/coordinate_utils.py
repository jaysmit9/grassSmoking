def haversine_distance(coord1, coord2):
    """Calculate the great-circle distance between two points on the Earth."""
    from geopy.distance import geodesic
    return geodesic(coord1, coord2).meters

def meters_per_lon_degree(lat):
    """Calculate meters per longitude degree at a given latitude."""
    METERS_PER_LAT_DEGREE = 111000
    return METERS_PER_LAT_DEGREE * np.cos(np.radians(lat))

def lat_lon_to_xy(lat, lon):
    """Convert latitude and longitude to Cartesian coordinates (x, y)."""
    x = lon * meters_per_lon_degree(lat)
    y = lat * 111000  # Approximate conversion for latitude
    return x, y

def xy_to_lat_lon(x, y, lat_reference):
    """Convert Cartesian coordinates (x, y) back to latitude and longitude."""
    lon = x / meters_per_lon_degree(lat_reference)
    lat = y / 111000  # Approximate conversion for latitude
    return lat, lon

def calculate_bearing(coord1, coord2):
    """Calculate the bearing from coord1 to coord2."""
    lat1, lon1 = np.radians(coord1)
    lat2, lon2 = np.radians(coord2)
    
    dlon = lon2 - lon1
    x = np.sin(dlon) * np.cos(lat2)
    y = np.cos(lat1) * np.sin(lat2) - (np.sin(lat1) * np.cos(lat2) * np.cos(dlon))
    
    initial_bearing = np.arctan2(x, y)
    # Convert from radians to degrees
    initial_bearing = np.degrees(initial_bearing)
    # Normalize to 0-360
    compass_bearing = (initial_bearing + 360) % 360
    
    return compass_bearing