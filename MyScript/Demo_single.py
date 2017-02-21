#!/usr/bin/env python
# -*- coding:utf-8 -*-

import logging
from dronekit import connect, VehicleMode, LocationGlobalRelative
from takeoff import *

import sys

sys.path.append("..")

from drone_network import Networking
from collision_avoidance import CollisionThread

connection_string = 'tcp:127.0.0.1:5763'

# Connect to the Vehicle
print 'Connecting to vehicle1 on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

# Create the interface with UDP broadcast sockets
debug = True
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

time.sleep(10)
# Close vehicle object before exiting script
print "\nClose vehicle object"
vehicle.close()

print "Completed"
