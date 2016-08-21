"""
Check a stopped vehicle's attributes by running sitl 
"""
from params import Params

import time, math, sys, socket, json, threading, select
from collections import namedtuple
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil 

MAX_DRONES = 15
#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not args.connect:
    print "Starting copter simulator (SITL)"
    from dronekit_sitl import SITL
    sitl = SITL()
    sitl.download('copter', '3.3', verbose=True)
    sitl_args = ['-I0', '--model', 'quad', '--home=-35.363261,149.165230,584,353']
    sitl.launch(sitl_args, await_ready=True, restart=True)
    connection_string='tcp:127.0.0.1:5760'


# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

self_params = Params(vehicle)

print self_params.velocity
print "Type before: ", type(self_params.ekf_ok)
self_params.set('battery_level', 50)
print self_params.battery_level
self_params.ekf_ok = True
print "Type after: ", type(self_params.ekf_ok)
print self_params.ekf_ok