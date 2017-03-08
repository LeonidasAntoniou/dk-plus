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
        self.leadForce_K = 1
        self.FormationForce_K = 0.5
        self.targetLocation = None
        self.FormationPosition = None

    def setFormation(self, formation_set):
        # Because the range for the operation is quite small, we use the specific latitude
        # 116.3397540 means the latitude for SYS
        earth_radius = 6378137.0  # Radius of "spherical" earth
        for i in range(0, formation_set.shape[0]):
            for j in range(0, formation_set.shape[1]):
                if i == 0:
                    # Lat
                    formation_set[i, j] = (formation_set[i, j] / earth_radius) * 180 / math.pi
                if i == 1:
                    # Lon
                    formation_set[i, j] = (formation_set[i, j] / (
                        earth_radius * math.cos(math.pi * 39.9793234) / 180)) * 180 / math.pi
        self.FormationPosition = np.matrix(formation_set)
        # # Coordinate offsets in radians
        # dLat = dNorth / earth_radius
        # dLon = dEast / (earth_radius * math.cos(math.pi * lat / 180))
        #
        # # New position in decimal degrees
        # newlat = lat + (dLat * 180 / math.pi)
        # newlon = lon + (dLon * 180 / math.pi)
        # self.FormationPosition = np.matrix(set)

    def getPosition(self, teammate):
        x = self.get_cenPos(teammate)[0]
        y = self.get_cenPos(teammate)[1]
        z = self.get_cenPos(teammate)[2]

        Vx = self.get_cenVel(teammate)[0]
        Vy = self.get_cenVel(teammate)[1]
        Vz = self.get_cenVel(teammate)[2]

        if (Vx > 0 and Vy > 0) or (Vx < 0 and Vy > 0):
            theta = math.pi / 2 - math.atan(Vx / Vy)
        else:
            theta = - math.atan(Vx / Vy)

        phi = math.atan(Vz / math.sqrt(Vx ** 2 + Vy ** 2))

        Rotaz = np.matrix(
            [[math.cos(theta), math.sin(theta), 0],
             [-math.sin(theta), math.cos(theta), 0],
             [0, 0, 1]])

        Rotax = np.matrix(
            [[1, 0, 0],
             [0, math.cos(phi), -math.sin(phi)],
             [0, math.sin(phi), math.cos(phi)]])

        c = self.FormationPosition[:, int(self.network.vehicle_params.SYSID_THISMAV - 1)].reshape(3, 1)
        abPos = Rotaz * Rotax * c + np.matrix([x, y, z]).reshape(3, 1)

        return list(np.array(abPos).ravel())

    def set_target_Loc(self, lat, lon, dNorth=-100, dEast=20):
        # self.targetLocation = get_location_metres(self.network.vehicle_params.global_lat,
        #                                           self.network.vehicle_params.global_lon,
        #                                           self.network.vehicle_params.global_alt, dNorth, dEast)

        self.targetLocation = get_location_metres(lat,
                                                  lon,
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

        # For now, no force on the altitude
        dampForce[-1] = 0
        logging.debug("Damp Force: %s", dampForce)
        return dampForce

    def LeadForce(self, teammate, single):

        if len(teammate) == 0 or single:
            cenPos = np.array([self.network.vehicle_params.global_lat,
                               self.network.vehicle_params.global_lon])
            cenAlt = np.array([self.network.vehicle_params.global_alt])
        else:
            cenPos = self.get_cenPos(teammate)[0:2]
            cenAlt = self.get_cenPos(teammate)[-1]

        tarPos = np.array([self.targetLocation.lat,
                           self.targetLocation.lon])

        tarAlt = np.array([self.targetLocation.lat])

        force = self.leadForce_K * (tarPos - cenPos) / np.linalg.norm(tarPos - cenPos)

        if np.linalg.norm(force) > self.MaxLeadForce:
            force = force * self.MaxLeadForce / np.linalg.norm(force)

        # For now, no force on the altitude
        force = np.append(force, 0)

        logging.debug("central_Position: %s ; target_Position: %s ;", self.get_cenPos(teammate)[0:2], tarPos)
        logging.debug("Lead force: %s", force)
        return force

    def FormationForce(self, teammate, single):
        if len(teammate) == 0 or single:
            FormationForce = 0
        else:
            ownPos = np.array([self.network.vehicle_params.global_lat,
                               self.network.vehicle_params.global_lon,
                               self.network.vehicle_params.global_alt])

            Formation_position = self.getPosition(teammate)

            FormationForce = self.FormationForce_K * (Formation_position - ownPos)
            # For now no force on Altitude
            FormationForce[-1] = 0
        return FormationForce

    def TotalForce(self, teammate, single):

        force = self.FormationForce(teammate, single) + self.LeadForce(teammate, single) + self.DampForce()

        if np.linalg.norm(force) > self.MaxForce:
            force = force * self.MaxForce / np.linalg.norm(force)
        logging.debug("Total force: %s ", force)
        return force

    def SendVelocity(self, teammate, single):
        add_vel = self.TotalForce(teammate, single) * self.network.POLL_RATE
        velocity = np.array(self.network.vehicle_params.velocity) + add_vel

        # For now Vz=0
        velocity[-1] = 0.0

        logging.debug("Current Velociy: %s", self.network.vehicle_params.velocity)
        logging.debug("Add velocity: %s ", add_vel)

        return list(velocity)
