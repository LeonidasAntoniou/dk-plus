from dronekit import LocationGlobal, LocationGlobalRelative
import math


def get_location_metres(lat, lon, alt, dNorth, dEast, dDown=None):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * lat / 180))

    # New position in decimal degrees
    newlat = lat + (dLat * 180 / math.pi)
    newlon = lon + (dLon * 180 / math.pi)
    if Down is None:
        return LocationGlobal(newlat, newlon, alt)
    else:
        return newlat, newlon, alt + dDown


def get_distance_metres(lat1, lon1, lat2, lon2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = 0
    dlong = 0

    dlat = lat2 - lat1
    dlong = lon2 - lon1

    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def distance_to_current_waypoint(vehicle):
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == 0:
        return None
    missionitem = vehicle.commands[nextwaypoint - 1]  # commands are zero indexed
    target_lat = missionitem.x
    target_lon = missionitem.y
    current_lat = vehicle.location.global_frame.lat
    current_lon = vehicle.location.global_frame.lon
    distancetopoint = get_distance_metres(current_lat, current_lon, target_lat, target_lon)
    return distancetopoint
