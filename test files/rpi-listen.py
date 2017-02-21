"""
Deployed in RPi's to check listening/updating/collision functionality
"""

import socket, select, threading, time, math
import cPickle as pickle
from params import Params

MAX_DRONES = 15
MAX_STAY = 4  # seconds
SAFETY_ZONE = 40  # meters
CRITICAL_ZONE = 10  # meters
HEADING_TOLERANCE = 5  # degrees

# Set the socket parameters
addr = ('<broadcast>', 54545)  # host, port

# Create socket and bind to address
sock_listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_listen.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock_listen.bind(addr)

params = []
self_params = {'lat': -35.3628118424, 'lon': 149.165780676, 'alt': 15, 'heading': 120}


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2['lat'] - aLocation1['lat']
    dlong = aLocation2['lon'] - aLocation1['lon']
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def update_params(message):
    global params
    found = False

    # First element
    if len(params) == 0:
        params.append(message)

    # Update values if message comes from the same sender or insert new sender
    else:
        for i in range(0, len(params)):
            if (params[i]).ID == message.ID:
                params[i] = message
                # params[i].print_all()
                found = True
                break
        if found == False:
            params.append(message)
        # params[i+1].print_all()

    # Remove entries that have not been updated the last four seconds
    params = [item for item in params if time.time() - item.last_recv <= MAX_STAY]


def collision():
    global params
    t_collision = threading.Timer(1.0, collision)
    t_collision.start()

    if len(params) == 0:
        return

    else:

        # From the detected drones, add the ones  within a 40-metre range
        near = [item for item in params if (get_distance_metres( \
            {"lat": self_params['lat'], "lon": self_params['lon']}, \
            {"lat": item.global_lat, "lon": item.global_lon}) <= SAFETY_ZONE) \
                & (get_distance_metres( \
            {"lat": self_params['lat'], "lon": self_params['lon']}, \
            {"lat": item.global_lat, "lon": item.global_lon}) > CRITICAL_ZONE) \
                & (abs(self_params['alt'] - item.global_alt) <= SAFETY_ZONE)]

        # From the detected drones, add the ones within a 10-metre range
        critical = [item for item in near if (get_distance_metres( \
            {"lat": self_params['lat'], "lon": self_params['lon']}, \
            {"lat": item.global_lat, "lon": item.global_lon}) <= CRITICAL_ZONE) \
                    & (abs(self_params['alt'] - item.global_alt) <= CRITICAL_ZONE)]

        """A premature check for the possibility of collision"""
        ## * * * Not that simple! * * * Check dot/scalar products
        # Collision is most probable if the opponent's heading has at most -180 degrees difference with the drone's heading
        # Some tolerance is taken into account due to autopilot/weather conditions
        # Collision is impossible to happen if opponent's heading is between drone's heading and drone's heading + 180 degrees
        collide = [item for item in near if
                   (item.heading >= abs(abs((self_params['heading'] - HEADING_TOLERANCE) - 180) - 360) & \
                    item.heading <= (self_params['heading'] + HEADING_TOLERANCE))]

        # for i in range(0, len(near)):
        # print "Drone approaching! ID: ", (near[i]).ID
        # set priorities
        # take action

        # for i in range(0, len(critical)):
        # print "Drone too close!!!! ID: ", (critical[i]).ID
        # all halt

        for i in range(0, len(collide)):
            print "Drone probable to collide!!! ID: ", (collide[i]).ID
            print "Opponent drone: Lat %s, Lon %s, Alt %s, Heading %s" % (
            (collide[i]).global_lat, (collide[i]).global_lon, \
            (collide[i]).global_alt, (collide[i]).heading)
            print "Our drone: Lat %s, Lon %s, Alt %s, Heading %s" % (self_params['lat'], self_params['lon'], \
                                                                     self_params['alt'], self_params['heading'])


def listen():
    t_listen = threading.Timer(1.0, listen)
    t_listen.start()

    ready = select.select([sock_listen], [], [], 1.0)
    if ready[0]:
        d = sock_listen.recvfrom(4096)
        raw_msg = d[0]
        try:
            msg = pickle.loads(raw_msg)

        except pickling.UnpicklingError:
            msg = raw_msg

        sender_addr = d[1]
        sender_ip = (d[1])[0]
        print "From addr:", sender_addr
        msg.ID = sender_ip
        msg.last_recv = time.time()
        msg.distance_from_self = get_distance_metres({"lat": self_params['lat'], "lon": self_params['lon']}, \
                                                     {"lat": msg.global_lat, "lon": msg.global_lon})
        print "Distance from self: ", msg.distance_from_self, "meters"
        update_params(msg)
    # if sock_listen.sendto("ACK", addr):
    # print "ACK sent to ", addr
    # else:
    # print "Nothing sent..."


listen()
collision()
