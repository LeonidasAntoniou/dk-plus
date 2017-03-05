#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2017/3/5
# @Author  : Leon.Nie
# @Site    : 
# @File    : formation.py
from geo_tools import *
import numpy as np
import logging


class Formation:
    def __init__(self, network):
        self.network = network
        self.MaxLeadForce = 10
        self.MaxForce = 10
        self.dampForce_K = -1.0
        self.leadForce_K = 0.2
        self.targetLocation = None

    def set_target_Loc(self, dNorth=-150, dEast=20):
        self.targetLocation = get_location_metres(self.network.vehicle_params.global_lat,
                                                  self.network.vehicle_params.global_lon,
                                                  self.network.vehicle_params.global_alt, dNorth, dEast)
        logging.info("Target Location set: %s", self.targetLocation)

    def get_target_Loc(self):
        return self.targetLocation

    def get_distance2target(self):
        return get_distance_metres(self.network.vehicle_params.global_lat, self.network.vehicle_params.global_lon,
                                   self.targetLocation.lat, self.targetLocation.lon)

    def get_cenPos(self, teammate):
        lat = self.network.vehicle_params.global_lat
        lon = self.network.vehicle_params.global_lon
        alt = self.network.vehicle_params.global_alt
        for drone in teammate:
            lat += drone.global_lat
            lon += drone.global_lon
            alt += drone.global_alt
        c_lat = lat / float(len(teammate) + 1)
        c_lon = lon / float(len(teammate) + 1)
        c_alt = alt / float(len(teammate) + 1)

        return c_lat, c_lon, c_alt

    def get_cenVel(self, teammate):
        velocity = self.network.vehicle_params.velocity
        for drone in teammate:
            velocity = [drone.velocity[i] + velocity[i] for i in range(len(velocity))]
        cenVel = [x / float(len(teammate) + 1) for x in velocity]

        return cenVel

    def DampForce(self):
        dampForce = self.dampForce_K * np.array(self.network.vehicle_params.velocity)
        return dampForce

    def LeadForce(self):
        ownPos = np.array([self.network.vehicle_params.global_lat,
                           self.network.vehicle_params.global_lon,
                           self.network.vehicle_params.global_alt])
        tarPos = np.array([self.targetLocation.lat,
                           self.targetLocation.lon,
                           self.targetLocation.alt])
        force = self.leadForce_K * (tarPos - ownPos)
        if np.linalg.norm(force) > self.MaxLeadForce:
            force = force * self.MaxLeadForce / np.linalg.norm(force)
        return force

    def FormationForce(self):
        pass
        return 0

    def TotalForce(self):
        force = self.FormationForce() + self.LeadForce() + self.DampForce()
        if np.linalg.norm(force) > self.MaxForce:
            force = force * self.MaxForce / np.linalg.norm(force)
        return force

    def SendVelocity(self):
        velocity = self.TotalForce() * self.network.POLL_RATE
        return list(velocity)
