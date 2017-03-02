#!/usr/bin/env python
# -*- coding:utf-8 -*-

import logging
from dronekit import connect
from pymavlink import mavutil  # Needed for command message definitions
from takeoff import *

import sys

sys.path.append("..")

from drone_network import Networking
from collision_avoidance import CollisionThread

logging.basicConfig(level=logging.INFO)


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting
    the yaw using this function there is no way to return to the default yaw "follow direction
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see:
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


# Set up option parsing to get connection string
# import argparse
#
# parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
# parser.add_argument('--connect',
#                     help="Vehicle connection target string. If not specified, SITL automatically started and used.")
# args = parser.parse_args()
#
# connection_string = args.connect
# sitl = None
#
# # Start SITL if no connection string specified
# if not connection_string:
#     import dronekit_sitl
#
#     sitl = dronekit_sitl.start_default()
#     connection_string = sitl.connection_string()
# logging.basicConfig(level=logging.INFO)

connection_string = 'tcp:192.168.6.46:5763'
# Connect to the Vehicle
print 'Connecting to vehicle1 on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

# Create the interface with UDP broadcast sockets
debug = True
address = ("192.168.6.255", 54545)
network = Networking(address, "UDP_BROADCAST", vehicle, debug)

# Add collision avoidance algorithm
t_collision = CollisionThread(network, debug=debug)

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

arm_and_takeoff(vehicle, 10)

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

while vehicle.mode.name != "RTL":
    time.sleep(1)

# Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

# Close broadcast thread and socket
logging.info("Close sockets")
network.stop()

print "Completed"
