#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2017/3/5
# @Author  : Leon.Nie
# @Site    : 
# @File    : Demo_formation.py

"""
This script is to test the APF method for only one UAV.
"""

import logging, time
from dronekit import connect, VehicleMode
from pymavlink import mavutil  # Needed for command message definitions

import sys

sys.path.append("..")

from drone_network import Networking
from collision_avoidance import CollisionThread
from act_tool import arm_and_takeoff

logging.basicConfig(level=logging.DEBUG)

# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(filename)s[line:%(lineno)d] %(levelname)s %(message)s',
                    datefmt='%a, %d %b %Y %H:%M:%S',
                    filename='my.log',
                    filemode='w')

# connection_string = 'tcp:192.168.6.46:5763'

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

# Create the interface with UDP broadcast sockets
debug = False
address = ("192.168.6.255", 54545)
network = Networking(address, "UDP_BROADCAST", vehicle, debug)

# Add collision avoidance algorithm
single = True
t_collision = CollisionThread(network, algorithm='formation', single=single, debug=debug)

# Get all vehicle attributes (state)
print "\nGet all vehicle attribute values:"
print " Autopilot Firmware version: %s" % vehicle.version
print "System ID：%s" % vehicle.parameters['SYSID_THISMAV']

# Set the targetLocation for the team
t_collision.formation.set_target_Loc(dNorth=-50, dEast=20)

arm_and_takeoff(vehicle, 10)

logging.info("Initializing interface")
network.run()

logging.info("Starting collision avoidance scheme")
t_collision.start()

print "Press any key to exit script"
exit = raw_input()
