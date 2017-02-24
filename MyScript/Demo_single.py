#!/usr/bin/env python
# -*- coding:utf-8 -*-

import logging
from dronekit import connect, VehicleMode, LocationGlobalRelative
from guided_set_speed_yaw import condition_yaw, send_ned_velocity
from takeoff import *

import sys

sys.path.append("..")

from drone_network import Networking
from collision_avoidance import CollisionThread

connection_string = 'tcp:127.0.0.1:5763'
logging.basicConfig(level=logging.INFO)

# Connect to the Vehicle
print 'Connecting to vehicle1 on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

# Create the interface with UDP broadcast sockets
debug = True
address = ("192.168.6.255", 54545)
network = Networking(address, "UDP_BROADCAST", vehicle, debug)

# Add collision avoidance algorithm
t_collision = CollisionThread(network, 'priorities', debug)

# Get all vehicle attributes (state)
print "\nGet all vehicle attribute values:"
print " Autopilot Firmware version: %s" % vehicle.version
print "System ID：%s" % vehicle.parameters['SYSID_THISMAV']

logging.info("Initializing interface")
network.run()

logging.info("Starting collision avoidance scheme")
t_collision.start()

"""
Fly the vehicle in a SQUARE path using velocity vectors (the underlying code calls the
SET_POSITION_TARGET_LOCAL_NED command with the velocity parameters enabled).

The thread sleeps for a time (DURATION) which defines the distance that will be travelled.

The code also sets the yaw (MAV_CMD_CONDITION_YAW) using the `set_yaw()` method in each segment
so that the front of the vehicle points in the direction of travel
"""

# Set up velocity vector to map to each direction.
# vx > 0 => fly North
# vx < 0 => fly South
NORTH = 2
SOUTH = -2

# Note for vy:
# vy > 0 => fly East
# vy < 0 => fly West
EAST = 2
WEST = -2

# Note for vz:
# vz < 0 => ascend
# vz > 0 => descend
UP = -0.5
DOWN = 0.5

DURATION = 20

# Square path using velocity
print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and velocity parameters")

print("Yaw 180 absolute (South)")
condition_yaw(180)

print("Velocity South & up")
send_ned_velocity(SOUTH, 0, UP, DURATION)
send_ned_velocity(0, 0, 0, 1)

print("Yaw 270 absolute (West)")
condition_yaw(270)

print("Velocity West & down")
send_ned_velocity(0, WEST, DOWN, DURATION)
send_ned_velocity(0, 0, 0, 1)

print("Yaw 0 absolute (North)")
condition_yaw(0)

print("Velocity North")
send_ned_velocity(NORTH, 0, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

print("Yaw 90 absolute (East)")
condition_yaw(90)

print("Velocity East")
send_ned_velocity(0, EAST, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)


"""
The example is completing. Return to home location.
"""
print("Setting RTL mode...")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()


# Close broadcast thread and socket
logging.info("Close sockets")
network.stop()

# Close vehicle object before exiting script
print "\nClose vehicle object"
vehicle.close()

print "Completed"
