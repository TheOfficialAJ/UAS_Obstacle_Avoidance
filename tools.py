import math
from dronekit import LocationGlobal
import numpy as np

import pyproj


def get_location_metres(original_location, dNorth, dEast):
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
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def get_3d_distance(aLocation1, aLocation2):  # Gives the 3d distance between two LocationRelativeGlobal objects
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    dalt = aLocation2.alt - aLocation1.alt
    return math.sqrt(((dlat * dlat) + (dlong * dlong)) * (1.113195e5 ** 2) + (dalt * dalt))


def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    geodesic = pyproj.Geod(ellps='WGS84')
    long1 = aLocation1.lon
    long2 = aLocation2.lon
    lat1 = aLocation1.lat
    lat2 = aLocation2.lat
    fwd_azimuth, back_azimuth, distance = geodesic.inv(long1, lat1, long2, lat2)
    return fwd_azimuth
    # off_x = aLocation2.lon - aLocation1.lon
    # off_y = aLocation2.lat - aLocation1.lat
    # bearing = 90.00 + np.degrees(np.arctan2(-off_y, off_x))
    # if bearing < 0:
    #     bearing += 360.00
    # return bearing
