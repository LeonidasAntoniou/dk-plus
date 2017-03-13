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
        self.FormationForce_K = 10e3
        self.TeamHomeLocation = None
        self.targetLocation = None
        self.FormationPosition = None

    def setFormation(self, formation_set):
        self.FormationPosition = np.matrix(formation_set)

    def set_target_Loc(self, lat, lon, alt, dNorth, dEast):
        """
        Target Location. Set before taking off
        :param lat: original latitude
        :param lon: original longitude
        :param alt: Target altitude
        :param dNorth:
        :param dEast:
        :return:
        """
        self.TeamHomeLocation = np.array([lat, lon, 0])

        self.targetLocation = get_location_metres(lat,
                                                  lon,
                                                  alt, dNorth, dEast)

        logging.info("Target Location set: %s", self.targetLocation)

    def get_target_Loc(self):
        return self.targetLocation

    def get_distance2target(self):
        return get_distance_metres(self.network.vehicle_params.global_lat, self.network.vehicle_params.global_lon,
                                   self.targetLocation.lat, self.targetLocation.lon)

    def getPosition(self, teammate):
        x, y, z = self.get_cenPos(teammate)

        Vx, Vy, Vz = self.get_cenVel(teammate)

        if (Vx > 0 and Vy > 0) or (Vx > 0 and Vy < 0):
            theta = math.pi / 2 - math.atan(Vy / Vx)
        else:
            if Vy != 0:
                theta = math.pi + math.atan(Vx / Vy)
            else:
                theta = math.pi + math.pi / 2

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
        abPos = np.array(Rotaz * Rotax * c).ravel()

        logging.debug("Local Formation Position : %s", abPos)

        Pos = np.array(get_location_formation(x,
                                              y,
                                              z, abPos[0], abPos[1], abPos[2]))
        logging.debug("Global Formation Position : %s", Pos)

        return Pos

    def get_cenPos(self, teammate):
        """
         Nearly proximate because we are running in a quite small range
        :param teammate:
        :return:
        """
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

        leadforce = self.leadForce_K * (tarPos - cenPos) / np.linalg.norm(tarPos - cenPos)

        if np.linalg.norm(leadforce) > self.MaxLeadForce:
            leadforce = leadforce * self.MaxLeadForce / np.linalg.norm(leadforce)

        # For now ,no force on altitude
        leadforce = np.append(leadforce, 0)

        logging.debug("Center_Position: %s ; Target_Position: %s ;", self.get_cenPos(teammate), tarPos)
        logging.debug("Lead force: %s", leadforce)

        return leadforce

    def FormationForce(self, teammate, single):
        if len(teammate) == 0 or single:
            FormationForce = 0
        else:
            ownPos = np.array([self.network.vehicle_params.global_lat,
                               self.network.vehicle_params.global_lon])

            ownAlt = np.array([self.network.vehicle_params.global_alt])

            Formation_position = self.getPosition(teammate)

            FormationForce = self.FormationForce_K * (Formation_position[0:2] - ownPos)

            # For now ,no force on altitude
            FormationForce = np.append(FormationForce, 0)

            logging.debug("Own Position: %s", ownPos)
            logging.debug("Formation force: %s ", FormationForce)

            # return 0 * FormationForce
        return FormationForce

    def TotalForce(self, teammate, single):

        LeadForce = self.LeadForce(teammate, single)
        FormationForce = self.FormationForce(teammate, single)
        DampForce = self.DampForce()

        force = LeadForce + FormationForce + DampForce

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
