#!/usr/bin/env python
# -*- coding:utf-8 -*-

from dronekit import connect,VehicleMode,LocationGlobalRelative
from takeoff import *

connection_string1 = 'tcp:192.168.6.46:5763'
connection_string2 = 'tcp:192.168.6.111:5763'
connection_string3 = 'tcp:192.168.6.27:5763'

# Connect to the Vehicle
print 'Connecting to vehicle1 on: %s' % connection_string1
vehicle1 = connect(connection_string1, wait_ready=True)
print 'Connecting to vehicle2 on: %s' % connection_string2
vehicle2 = connect(connection_string2, wait_ready=True)
print 'Connecting to vehicle3 on: %s' % connection_string3
vehicle3 = connect(connection_string3, wait_ready=True)

# Get all vehicle attributes (state)
print "\nGet all vehicle1 attribute values:"
print " Autopilot Firmware version: %s" % vehicle1.version
print "System ID：%s" % vehicle1.parameters['SYSID_THISMAV']

print "\nGet all vehicle2 attribute values:"
print " Autopilot Firmware version: %s" % vehicle2.version
print "System ID：%s" % vehicle2.parameters['SYSID_THISMAV']

print "\nGet all vehicle3 attribute values:"
print " Autopilot Firmware version: %s" % vehicle3.version
print "System ID：%s" % vehicle3.parameters['SYSID_THISMAV']

arm_and_takeoff(vehicle1,10)
arm_and_takeoff(vehicle2,10)
arm_and_takeoff(vehicle3,10)

print "Set default/target airspeed to 3"
vehicle1.airspeed = 3
vehicle2.airspeed = 3
vehicle3.airspeed = 3

print "Going towards first point for 10 seconds ..."
point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
vehicle1.simple_goto(point1)
vehicle2.simple_goto(point1)
vehicle3.simple_goto(point1)

# sleep so we can see the change in map
time.sleep(10)

# print "Going towards second point for 10 seconds (groundspeed set to 10 m/s) ..."
# point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
# vehicle1.simple_goto(point2, groundspeed=10)
# vehicle2.simple_goto(point2, groundspeed=10)
# vehicle3.simple_goto(point2, groundspeed=10)
#
# # sleep so we can see the change in map
# time.sleep(30)

print "Returning to Launch"
vehicle1.mode = VehicleMode("RTL")
vehicle2.mode = VehicleMode("RTL")
vehicle3.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print "\nClose vehicle object"
vehicle1.close()
vehicle2.close()
vehicle3.close()

print "Completed"
