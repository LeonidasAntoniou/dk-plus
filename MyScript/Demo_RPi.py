#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2017/2/27
# @Author  : Leon.Nie
# @File    : Demo_RPi.py
"""
This script is used to test wether it works well on the Raspberry3
which connect to the ardupilot Pixhawk via 3DR Radio
"""
import logging
from dronekit import connect

import sys

sys.path.append("..")

from drone_network import Networking
from collision_avoidance import CollisionThread

# connection_string = '/dev/ttyUSB0,57600'
# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(
    description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect',
                    help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
logging.basicConfig(level=logging.INFO)

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

# Create the interface with UDP broadcast sockets
debug = False
address = ("192.168.6.255", 54545)
network = Networking(address, "UDP_BROADCAST", vehicle, debug)

# Add collision avoidance algorithm
t_collision = CollisionThread(network)

# Get all vehicle attributes (state)
print "\nGet all vehicle attribute values:"
print " Autopilot Firmware version: %s" % vehicle.version
print "System ID：%s" % vehicle.parameters['SYSID_THISMAV']

logging.info("Initializing interface")
network.run()

logging.info("Starting collision avoidance scheme")
t_collision.start()

print "Press any key to exit script"
exit = raw_input()