#!/usr/bin/env python
# -*- coding:utf-8 -*-

from dronekit import connect,VehicleMode,LocationGlobalRelative
from takeoff import *

connection_string = '127.0.0.1:5763'

# Connect to the Vehicle
print 'Connecting to vehicle1 on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)


# Get all vehicle attributes (state)
print "\nGet all vehicle attribute values:"
print " Autopilot Firmware version: %s" % vehicle.version
print "System ID：%s" % vehicle.parameters['SYSID_THISMAV']