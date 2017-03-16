#!/usr/bin/env python
# -*- coding:utf-8 -*-

from dronekit import VehicleMode
import logging
import time


def arm_and_takeoff(vehicle, target_alt=10):
    """
    Arms vehicle and fly to target_altitude.
    """

    logging.info("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        logging.info(" Waiting for vehicle to initialise...")
        time.sleep(1)

    logging.info("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        logging.info(" Waiting for arming...")
        time.sleep(1)

    logging.info("Taking off!")
    vehicle.simple_takeoff(target_alt)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        logging.info(" Altitude: %s", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= target_alt * 0.95:
            logging.info("Reached target altitude")
            break
        time.sleep(1)
