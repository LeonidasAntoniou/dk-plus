import time, math, sys, socket, threading, select, rpdb2
from collections import namedtuple
from params_1_2 import Params
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import cPickle as pickle

SAFETY_ZONE = 40  # metres
CRITICAL_ZONE = 10  # metres

Geo = namedtuple("Geo", "lat lon")  # For argument passing in get_distance_metres

# Parameter dictionary
self_params = Params(dummy=True)
params_1 = Params(dummy=True)
params_2 = Params(dummy=True)
params_3 = Params(dummy=True)
params_4 = Params(dummy=True)

# Lists that keep parameter data.
# -params stores all data received
# -near stores all drones in params that are within a close range
# -critical stores all drones in near that are within critical range
params = []
near = []
critical = []


def collision():
    global near, critical

    while True:
        print params
        print near
        print critical

        # No drones in range
        if len(params) == 0:
            return

        else:

            # From the detected drones, add any within a 40-metre range
            near = [item for item in params if (get_distance_metres( \
                self_params.global_lat, self_params.global_lon, \
                item.global_lat, item.global_lon) <= SAFETY_ZONE) \
                    & (abs(self_params.global_alt - item.global_alt) <= SAFETY_ZONE)]

            # From the near drones, add any within a 10-metre range
            critical = [item for item in near if (get_distance_metres( \
                self_params.global_lat, self_params.global_lon, \
                item.global_lat, item.global_lon) <= CRITICAL_ZONE) \
                        & (abs(self_params.global_alt - item.global_alt) <= CRITICAL_ZONE)]

            """A premature check for the possibility of collision"""
            ## * * * Not that simple! * * * Check dot/scalar products
            # Collision is most probable if the opponent's heading has at most -180 degrees difference with the drone's heading
            # Some tolerance is taken into account due to autopilot/weather conditions
            # Collision is impossible to happen if opponent's heading is between drone's heading and drone's heading + 180 degrees
            # collide = [item for item in near if (item.heading >= abs(abs((self_params.heading - HEADING_TOLERANCE) - 180) - 360) & \
            #									   item.heading <= (self_params.heading + HEADING_TOLERANCE))]

            for i in range(0, len(near)):
                print "Drone approaching! ID: ", (near[i]).ID
            # set priorities
            # take action

            for i in range(0, len(critical)):
                print "Drone too close!!!! ID: ", (critical[i]).ID
            # all halt

            # for i in range(0, len(collide)):
            # print "Drone probable to collide!!! ID: ", (collide[i]).ID
            # print "Opponent drone: Lat %s, Lon %s, Alt %s, Heading %s" % ((collide[i]).global_lat,(collide[i]).global_lon, \
            # (collide[i]).global_alt, (collide[i]).heading)
            # print "Our drone: Lat %s, Lon %s, Alt %s, Heading %s" % (self_params.lat,self_params.lon, \
            # self_params.alt, self_params.heading)

        time.sleep(1)


def get_location_metres(lat, lon, alt, dNorth, dEast):
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
    print newlat, newlon
    return (newlat, newlon, alt)


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

    distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
    print "Distance:", distance
    return distance


t_collision = threading.Thread(target=collision)
t_collision.daemon = True
t_collision.start()

(params_1.global_lat, params_1.global_lon, params_1.global_alt) = get_location_metres(self_params.global_lat,
                                                                                      self_params.global_lon,
                                                                                      self_params.global_alt, 8, -8)
(params_2.global_lat, params_2.global_lon, params_2.global_alt) = get_location_metres(self_params.global_lat,
                                                                                      self_params.global_lon,
                                                                                      self_params.global_alt, 30, -30)
params.append(params_1)
params.append(params_2)

time.sleep(2)

(params_3.global_lat, params_3.global_lon, params_3.global_alt) = get_location_metres(self_params.global_lat,
                                                                                      self_params.global_lon,
                                                                                      self_params.global_alt, 5, -5)
(params_4.global_lat, params_4.global_lon, params_4.global_alt) = get_location_metres(self_params.global_lat,
                                                                                      self_params.global_lon,
                                                                                      self_params.global_alt, 60, -60)
params.append(params_3)
params.append(params_4)

time.sleep(2)
