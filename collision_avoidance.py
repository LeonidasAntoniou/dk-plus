"""
Handler for the collision avoidance part of the module.
A specific drone_network.Networking object needs to be running
Custom algorithms can be specified
Spawns a thread by instantiation

Author: Leonidas Antoniou 
mail: leonidas.antoniou@gmail.com
"""
from multiprocessing import Process
import threading, time, itertools, logging
import geo_tools as geo
from dronekit import VehicleMode, Command
from operator import attrgetter
from pymavlink import mavutil
from collections import namedtuple

Context = namedtuple('Context', ['mode', 'mission', 'next_wp'])


class CollisionThread(threading.Thread):
    def __init__(self, network, algorithm=None, debug=False):
        threading.Thread.__init__(self)
        self.daemon = True
        self.network = network
        self.near = []
        self.critical = []
        self.algorithm = algorithm
        self.in_session = False
        self.context = None
        self.debug = debug

        self.update_proc = Process(target=self.update_drone_list)
        self.priority_proc = Process(target=self.give_priorities)

    def run(self):
        # Deploy your collision avoidance algorithm here
        while True:
            if self.algorithm == None:
                self.no_protocol()

            elif self.algorithm == 'priorities':
                self.priorities_protocol()

            else:
                pass

            time.sleep(0.5)

    """A Collision Avoidance API"""

    def no_protocol(self):
        # What to do if no protocol is specified
        # Currently it just outputs the drones in vicinity

        # self.update_proc.start()
        # self.update_proc.join()
        self.update_drone_list()

        self.print_drones_in_vicinity()

    def priorities_protocol(self):
        # A naive protocol based in priorities:
        # Stops every drone whose priority is other than '1'
        # Drone with priority '1' continues mission/movement
        # Currently works for AUTO mode

        # Give priorities
        # self.priority_proc.start()
        # self.priority_proc.join()
        self.give_priorities()

        # self.update_proc.start()
        # self.update_proc.join()
        self.update_drone_list()

        self.print_drones_in_vicinity()

        # Nothing to do if no drones are around and drone is not in avoidance state
        if len(self.near) == 0 and not self.in_session:
            return

        # Get priority number
        priority_num = self.get_priority_num()

        # Perform actions
        self.in_session = True
        if priority_num == 1:
            self.give_control()

        else:
            self.take_control()

    def give_priorities(self):
        """
        -If drone is grounded then its priority is low, exempting mode 'ARMED'
        -System status needs to be checked first, if it's critical it has the greatest capability of all
        -----If status is OK then highest priority is given to highest mission importance first
        ---------Among same mission importances, highest priority is given to the ones with less capabilities
        ------------If they have the same status, mission importance, capabilities fine-tune with battery level
        """

        # Temporary priority lists
        top = []
        high = []
        medium = []
        low = []

        # Depending the autopilot. The following modes are found in ardupilot
        grounded_state = ['UNINIT', 'BOOT', 'POWEROFF', 'STANDBY', 'CALIBRATING', 'LOCKED']
        auto_modes = ['RTL', 'LOITER', 'AUTO', 'GUIDED', 'CIRCLE', 'LAND', 'BRAKE', 'LIFTOFF']
        manual_modes = ['ALT_HOLD', 'STABILIZE', 'MANUAL', 'ACRO', 'SPORT', 'DRIFT', 'POSHOLD', 'SIMPLE',
                        'SUPER_SIMPLE']

        # Assign each drone to its priority list
        priority_num = 1
        for drone in self.network.drones:
            has_capabilities = drone.set_global_alt or drone.set_attitude
            has_mayday = (drone.system_status == 'CRITICAL') or (drone.system_status == 'EMERGENCY')

            # Manual flying drones without capabilities
            # Drones in Emergency/Critical state
            if (
                                drone.mode in manual_modes and not has_capabilities and drone.system_status not in grounded_state) or has_mayday:
                top.append(drone)

            # Top importance drones in flying state or ready to fly
            elif (drone.mission_importance == 2 and drone.system_status not in grounded_state or
                              drone.mission_importance == 2 and drone.system_status not in grounded_state):
                high.append(drone)

            # Drones not in level-2 importance
            # Drones in flying state or armed with remote capabilities
            # Drones flying or armed in one of the automatic flying modes
            elif ((drone.mode in auto_modes and drone.system_status not in grounded_state) or
                      (drone.mode in manual_modes and has_capabilities and drone.system_status not in grounded_state)):
                medium.append(drone)

            # Grounded drones with low importance
            elif drone.system_status in grounded_state:
                low.append(drone)

            # Assign priority number, redundant because list is sorted
            # based on priorities (top to lowest) but you never know...
            drone.priority = priority_num
            priority_num = priority_num + 1

        # Sort each sublist based on mission importance (top and high priority lists don't need that)
        medium.sort(key=attrgetter('mission_importance'), reverse=True)
        low.sort(key=attrgetter('mission_importance'), reverse=True)

        # Fine-tune sorting with battery level
        top.sort(key=attrgetter('battery_level'))
        high.sort(key=attrgetter('battery_level'))
        medium.sort(key=attrgetter('battery_level'))
        low.sort(key=attrgetter('battery_level'))

        # Combine everything back to drones list
        drones = [top, high, medium, low]
        self.network.drones = list(itertools.chain.from_iterable(drones))

        if self.debug:
            # Print priorities
            print "Printing Priorities:"
            for drone in self.network.drones:
                print "ID:", drone.ID, " Priority: ", drone.priority

    def take_control(self):
        """Changes speed to zero by changing mode and overriding the RC3 channel"""

        if self.network.vehicle.mode.name == 'POSHOLD':
            # Already in POSHOLD mode
            pass
        else:
            # Save context
            self.save_context()

            # Change mode and assert
            self.network.vehicle.mode = VehicleMode("POSHOLD")
        # while self.network.vehicle.mode.name != "POSHOLD":
        # time.sleep(0.2)

        # self.network.vehicle.mode = VehicleMode("GUIDED")
        # while self.network.vehicle.mode.name != "POSHOLD":
        #   time.sleep(0.5)

        # Give RC command so that we can bypass RC failsafe, 1500 means stay steady
        self.network.vehicle.channels.overrides['3'] = 1500  # throttle
        logging.info("Control taken!")

    def give_control(self):
        """Gives control by restoring to pre-avoidance state"""

        if self.context != None:
            self.restore_context()

        # Cancel RC override
        self.network.vehicle.channels.overrides['3'] = None

        # End session
        self.in_session = False
        logging.info("Control given!")

    def save_context(self):
        """Currently keeping information about mode and mission"""

        # Save context: mode, mission
        mode_name = self.network.vehicle.mode.name
        cur_mission = self.current_mission()
        next_waypoint = self.network.vehicle.commands.next
        self.context = Context(mode_name, cur_mission, next_waypoint)

    def restore_context(self):
        """Returns to the state before collision avoidance handling"""

        # Set to GUIDED mode to add any new commands
        if self.network.vehicle.mode.name != 'GUIDED':
            self.network.vehicle.mode = VehicleMode("GUIDED")
        # while self.network.vehicle.mode.name != "GUIDED": #Wait until mode is changed
        # time.sleep(0.5)

        # Save x, y, z values of mission waypoints in lists since commands.clear()
        # seems to clear every instance of CommandSequence object
        x = []
        y = []
        z = []

        for wp in self.context.mission:
            x.append(wp.x)
            y.append(wp.y)
            z.append(wp.z)

        cmds = self.network.vehicle.commands
        cmds.clear()

        # Add pre-avoidance context:
        # Waypoints
        for i in range(0, len(x)):
            cmds.add(
                Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 0, 0, 0, 0, 0, x[i], y[i], z[i]))

        cmds.upload()

        # Next waypoint
        self.network.vehicle.commands.next = self.context.next_wp

        # Flight mode
        self.network.vehicle.mode = VehicleMode(self.context.mode)

    # while self.network.vehicle.mode.name != self.context.mode:
    # time.sleep(0.5)

    def update_drone_list(self):

        # Empty previous list components
        self.near[:] = []
        self.critical[:] = []

        # 1.Remove entries that have not been updated for the last MAX_STAY seconds
        self.network.drones = [item for item in self.network.drones if \
                               (item.last_recv == None) \
                               or (time.time() - item.last_recv <= self.network.MAX_STAY)]

        # 2.Update own vehicle parameter entry in the right position
        for i in range(0, len(self.network.drones)):
            if self.network.drones[i].ID == self.network.vehicle_params.ID:
                self.network.drones[i] = self.network.vehicle_params
                break

        # 3.Update near-zone and critical-zone lists
        drone_list = self.network.drones

        # Only our drone is in the list
        if len(drone_list) == 1:
            pass

        else:
            # This value is slightly not concurrent
            own = self.network.vehicle_params

            # From the detected drones, add any within a SAFETY-to-CRITICAL-metre range
            self.near = [item for item in drone_list if \
                         (item.ID != own.ID)
                         & (geo.get_distance_metres(own.global_lat, own.global_lon, item.global_lat,
                                                    item.global_lon) <= self.network.SAFETY_ZONE) \
                         & (geo.get_distance_metres(own.global_lat, own.global_lon, item.global_lat,
                                                    item.global_lon) > self.network.CRITICAL_ZONE) \
                         & (abs(own.global_alt - item.global_alt) <= self.network.SAFETY_ZONE)]

            # From the near drones, add any within a CRITICAL-metre range
            self.critical = [item for item in drone_list if \
                             (item.ID != own.ID) \
                             & (geo.get_distance_metres(own.global_lat, own.global_lon, item.global_lat,
                                                        item.global_lon) <= self.network.CRITICAL_ZONE) \
                             & (abs(own.global_alt - item.global_alt) <= self.network.CRITICAL_ZONE)]

    def print_drones_in_vicinity(self):
        # Print drone IDs that are in close and critical range
        # Inform if no such drones are found
        if len(self.near) == 0 and len(self.critical) == 0:
            # print "No dangerous drones found"
            pass

        else:
            own_lat = self.network.vehicle_params.global_lat
            own_lon = self.network.vehicle_params.global_lon

            for drone in self.near:
                logging.info("Drone approaching! ID: %s", drone.ID)
                logging.info("Distance: %s",
                             geo.get_distance_metres(own_lat, own_lon, drone.global_lat, drone.global_lon))

            for drone in self.critical:
                logging.info("Drone too close!!!! ID: %s", drone.ID)

    def current_mission(self):
        # Retrieves current mission of vehicle
        cmds = self.network.vehicle.commands
        cmds.download
        cmds.wait_ready()
        return cmds

    def get_priority_num(self):
        priority_num = None
        for drone in self.network.drones:
            if drone.ID == self.network.vehicle_params.ID:
                priority_num = drone.priority
                break

        logging.info("Flying drone's priority number is: %s", priority_num)
        return priority_num
