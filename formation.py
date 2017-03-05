#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2017/3/5
# @Author  : Leon.Nie
# @Site    : 
# @File    : formation.py
from dronekit import LocationGlobal


class Formation:
    def __init__(self, network):
        self.network = network

    def get_cenPos(self, teammate):
        lat = self.network.vehicle_params.global_lat
        lon = self.network.vehicle_params.global_lon
        alt = self.network.vehicle_params.global_alt
        for drone in teammate:
            lat += drone.global_lat
            lon += drone.global_lon
            alt += drone.global_alt
        c_lat = lat / float(len(teammate))
        c_lon = lon / float(len(teammate))
        c_alt = alt / float(len(teammate))

        return LocationGlobal(c_lat, c_lon, c_alt)

    def get_cenVel(self, teammate):
        velocity = self.network.vehicle_params.velocity
        for drone in teammate:
            velocity += drone.velocity
        velocity / float(len(teammate))

        return velocity
