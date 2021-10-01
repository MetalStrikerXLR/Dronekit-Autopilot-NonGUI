import math
import time
import traceback
import threading
from enum import Enum

from dronekit import VehicleMode, LocationGlobalRelative, LocationGlobal, Command, connect
from pymavlink import mavutil

coord_dict = {
    "PTCL1": [float(33.6730629), float(73.0523336), float(20)],
    "PTCL2": [float(33.6729402), float(73.0520667), float(5)],
    "CASE1": [float(33.6940573), float(72.8235720), float(30)],
    "CASE2": [float(33.6938386), float(72.8231174), float(5)]
}


class States(Enum):
    """
    States of drone
    """
    INIT = 1
    IDLE = 2
    TAKEOFF = 3
    HOVER = 4
    RTL = 5
    AUTO = 6
    LAND = 7
    GOTO = 8


def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


class Autocopter(object):
    """
    This class consists of logic for states (methods 'do_*' & dependencies) and method 'new_command'
    """

    def __init__(self, _connection_string):
        self._vehicle = None
        self.connection_string = _connection_string

        # Params of auto-missions
        self._mission_created = False
        self._goto_location = None
        self._work_alt = 5
        self._need_hover = True

        # Params of states machine
        self.current_state = States.INIT
        self.stop_state = False
        self.next_state = States.IDLE

    def create_mission(self, lat, lon, alt):
        self._mission_created = False

        try:
            self._goto_location = LocationGlobalRelative(lat, lon, alt)
            # print 'Create a new mission (for current location)'
            self._adds_square_mission(self._vehicle.location.global_relative_frame, 6)
            self._mission_created = True
            return "Mission completed successfully!"
        except Exception as ex:
            self._mission_created = False
            return "An error occurred while building a mission" + str(ex) + "\n" + traceback.format_exc()
        finally:
            pass

    def new_state(self, new_state):
        self.stop_state = True
        self.next_state = new_state

    @property
    def get_location(self):
        return self._vehicle.location.global_frame.lat, self._vehicle.location.global_frame.lon

    @property
    def _status_of_connect(self):
        if self._vehicle is not None:
            return True
        else:
            raise Exception("Не удалось подключиться к APM")

    def disconnect(self):
        """
        Close vehicle object before exiting script
        :return:
        """
        if self._vehicle is not None:
            self._vehicle.close()
            print("Successful disconnect APM")

    @property
    def get_status(self):
        buf = "\nGPS: %s" % self._vehicle.gps_0 + \
              "\nBattery: %s" % self._vehicle.battery + \
              "\nLast Heartbeat: %s" % self._vehicle.last_heartbeat + \
              "\nIs Armable?: %s" % self._is_armable + \
              "\nSystem status: %s" % self._vehicle.system_status.state + \
              "\nMode: %s" % self._vehicle.mode.name + \
              "\nGlobal Location: %s" % self._vehicle.location.global_frame + \
              "\nGlobal Relative Location: %s" % self._vehicle.location.global_relative_frame + \
              "\nLocal Location: %s" % self._vehicle.location.local_frame + \
              "\nAttitude: %s" % self._vehicle.attitude + \
              "\nHeading: %s" % self._vehicle.heading + \
              "\nGroundspeed: %s" % self._vehicle.groundspeed + \
              "\nAirspeed: %s" % self._vehicle.airspeed
        return buf

    @property
    def _distance_to_current_waypoint(self):
        """
        Gets distance in metres to the current waypoint.
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint = self._vehicle.commands.next
        if nextwaypoint == 0:
            return None
        missionitem = self._vehicle.commands[nextwaypoint - 1]  # commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
        distancetopoint = get_distance_metres(self._vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint

    def download_mission(self):
        """
        Download the current mission from the vehicle.
        """
        cmds = self._vehicle.commands
        cmds.download()
        cmds.wait_ready()  # wait until download is complete.

    def _adds_square_mission(self, aLocation, aSize):
        """
        Adds a takeoff command and four waypoint commands to the current mission.
        The waypoints are positioned to form a square of side length 2*aSize around
        the specified LocationGlobal (aLocation).

        The function assumes vehicle.commands matches the vehicle mission state
        (you must have called download at least once in the session and after clearing the mission)
        """
        cmds = self._vehicle.commands
        print("Clear any existing commands")
        cmds.clear()
        print("Define/add new commands.")

        # Add new commands. The meaning/order of the parameters is documented in the Command class.

        # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,
                    0, 0,
                    0, 0, 0, 0, 10))
        # Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
        point1 = get_location_metres(aLocation, aSize, -aSize)
        point2 = get_location_metres(aLocation, aSize, aSize)
        point3 = get_location_metres(aLocation, -aSize, aSize)
        point4 = get_location_metres(aLocation, -aSize, -aSize)
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    0,
                    0, 0, 0, point1.lat, point1.lon, 11))
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    0,
                    0, 0, 0, point2.lat, point2.lon, 12))
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    0,
                    0, 0, 0, point3.lat, point3.lon, 13))
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    0,
                    0, 0, 0, point4.lat, point4.lon, 14))
        # add dummy waypoint "5" at point 4 (lets us know when have reached destination)
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    0,
                    0, 0, 0, point4.lat, point4.lon, 14))

        print("Upload new commands to vehicle")
        cmds.upload()

    @property
    def onLand(self):
        return self._vehicle.system_status.state == 'STANDBY'

    def do_INIT(self):
        self.current_state = States.INIT

        print("Autopilot is online")
        print("STATE = %s" % self.current_state)

        while True:
            try:
                print("Connecting to APM ...")
                self._vehicle = connect(self.connection_string, wait_ready=False)

                if self._status_of_connect:
                    print("Connect successful!")
                    next_state = States.IDLE
                    print(
                        "Successful completion of the state %s" % self.current_state + " switch to state %s" % next_state)
                    self.current_state = next_state
                    break

            except Exception as ex:
                print("Connection failed! " + str(ex))
                print("Timeout 10s")
                time.sleep(10)

    def do_LAND(self):
        self.current_state = States.LAND
        print('STATE = ' + str(self.current_state))
        self.stop_state = False
        self.next_state = States.IDLE
        # http://ardupilot.org/copter/docs/land-mode.html
        self._vehicle.mode = VehicleMode("LAND")
        while not self.onLand:
            if not self.stop_state:
                # print('Waiting for ' + self.current_state)
                time.sleep(1)
            else:
                print("Interrupt state %s" % self.current_state + " switch to state %s" % self.next_state)
                return self.next_state
        print("Successful completion of the state %s" % self.current_state + " switch to state %s" % self.next_state)
        return self.next_state

    def do_IDLE(self):
        self.current_state = States.IDLE
        print("STATE = %s" % self.current_state)
        self.stop_state = False
        self.next_state = States.IDLE
        # http://ardupilot.org/copter/docs/ac2_guidedmode.html
        # http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
        self._vehicle.armed = False
        self._vehicle.mode = VehicleMode("GUIDED")
        while True:
            if not self.stop_state:
                # print('I\'m in '+self.current_state)
                time.sleep(1)
            else:
                print("Interrupt state %s" % self.current_state + " switch to state %s" % self.next_state)
                return self.next_state

    def _simple_goto_wrapper(self, lat, lon, alt=20, groundspeed=7.5):
        # Задаем координаты нужной точки
        a_location = LocationGlobalRelative(lat, lon, alt)
        # полетели
        self._vehicle.simple_goto(a_location)
        # Путевая скорость, м/с
        self._vehicle.groundspeed = groundspeed

    def do_HOVER(self):
        self.current_state = States.HOVER
        print("STATE = %s" % self.current_state)
        self.stop_state = False
        self.next_state = States.HOVER
        self._vehicle.mode = VehicleMode("GUIDED")
        if self._need_hover:
            self._simple_goto_wrapper(self._vehicle.location.global_relative_frame.lat,
                                      self._vehicle.location.global_relative_frame.lon,
                                      self._vehicle.location.global_relative_frame.alt)
        self._need_hover = True  # сброс
        while True:
            if not self.stop_state:
                # print('I\'m in '+self.current_state)
                time.sleep(1)
            else:
                print("Interrupt state %s" % self.current_state + " switch to state %s" % self.next_state)
                return self.next_state

    def do_RTL(self):
        self.current_state = States.RTL
        print("STATE = %s" % self.current_state)
        self.stop_state = False
        self.next_state = States.IDLE
        # http://ardupilot.org/copter/docs/rtl-mode.html
        self._vehicle.mode = VehicleMode("RTL")
        while not self.onLand:
            if not self.stop_state:
                # print('Waiting for ' + self.current_state)
                time.sleep(1)
            else:
                print("Interrupt state %s" % self.current_state + " switch to state %s" % self.next_state)
                return self.next_state

        print("Successful completion of the state %s" % self.current_state + " switch to state %s" % self.next_state)
        return self.next_state

    @property
    def _is_armable(self):
        """
        Returns `True` if the vehicle is ready to arm, false otherwise (``Boolean``).

        This attribute wraps a number of pre-arm checks, ensuring that the vehicle has booted,
        has a good GPS fix, and that the EKF pre-arm is complete.
        """
        # check that mode is not INITIALSING
        # check that we have a GPS fix
        # check that EKF pre-arm is complete
        return self._vehicle.mode != 'INITIALISING' and self._vehicle.gps_0.fix_type > 1

    def do_TAKEOFF(self):
        aTargetAltitude = self._work_alt
        self.current_state = States.TAKEOFF
        print("STATE = %s" % self.current_state)
        self.stop_state = False
        self.next_state = States.HOVER
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        print("Waiting for vehicle to initialise...")
        while not self._is_armable:  # проверка не дронкита, а собственная
            if not self.stop_state:
                # print('Waiting for ' + self.current_state)
                # print('Waiting for vehicle to initialise...')
                time.sleep(1)
            else:
                print("Interrupt state %s" % self.current_state + " switch to state %s" % self.next_state)
                print("Stopping takeoff on pre-arm!")
                return self.next_state

        # Copter should arm in GUIDED mode
        self._vehicle.mode = VehicleMode("GUIDED")
        print("Arming motors")
        self._vehicle.armed = True
        while not self._vehicle.armed:
            if not self.stop_state:
                # print('Waiting for ' + self.current_state)
                # print('Waiting for arming...')
                time.sleep(1)
            else:
                print("Interrupt state %s" % self.current_state + " switch to state %s" % self.next_state)
                print("Stopping takeoff on arm!")
                return self.next_state

        print("Taking off!")
        self._vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
        # Wait until the vehicle reaches a safe height before processing the goto
        # (otherwise the command after Vehicle.simple_takeoff will execute immediately).
        while self._vehicle.location.global_relative_frame.alt < aTargetAltitude * 0.95:
            if not self.stop_state:
                # print('Waiting for ' + self.current_state)
                print("Altitude: %s" % self._vehicle.location.global_relative_frame.alt)
                time.sleep(1)
            else:
                print("Interrupt state %s" % self.current_state + " switch to state %s" % self.next_state)
                print("Stopping takeoff on fly!")
                return self.next_state

        print("Successful completion of the state %s" % self.current_state + " switch to state %s" % self.next_state)
        self._need_hover = False
        return self.next_state

    def _is_arrived(self, lat, lon, alt, precision=0.5):    # Safety
        # function taken from https://habrahabr.ru/post/281591/
        # current position
        veh_loc = self._vehicle.location.global_relative_frame
        # get data in meters
        diff_lat_m = (lat - veh_loc.lat) * 1.113195e5
        diff_lon_m = (lon - veh_loc.lon) * 1.113195e5
        diff_alt_m = alt - veh_loc.alt
        # deviation
        dist_xyz = math.sqrt(diff_lat_m ** 2 + diff_lon_m ** 2 + diff_alt_m ** 2)
        if dist_xyz < precision:
            # print("Arrived at the place")
            return True
        else:
            # print(Have not yet flown)
            return False

    def do_GOTO(self):
        self.next_state = States.HOVER  # should only be HOVER
        if self._mission_created:
            self.current_state = States.GOTO
            print("STATE = %s" % self.current_state)
            self.stop_state = False
            self.next_state = States.HOVER
            self._vehicle.mode = VehicleMode("GUIDED")
            self._simple_goto_wrapper(self._goto_location.lat, self._goto_location.lon, self._goto_location.alt)
            while not self._is_arrived(self._goto_location.lat, self._goto_location.lon, self._goto_location.alt):
                if not self.stop_state:
                    # print('I\'m in '+self.current_state)
                    print("To the destination: " + str(
                        get_distance_metres(self._goto_location, self._vehicle.location.global_relative_frame)) + "м")
                    time.sleep(1)
                else:
                    print("Interrupt state %s" % self.current_state + " switch to state %s" % self.next_state)
                    return self.next_state
            print(
                "Successful completion of the state %s" % self.current_state + " switch to state %s" % self.next_state)
            self._need_hover = False
            return self.next_state
        else:
            print(
                "Error: Create a mission beforehand! Now %s" % self.current_state + " switch to state %s" % self.next_state)
            return self.next_state

    def do_AUTO(self):
        self.next_state = self.current_state  # должен быть только HOVER
        if self._mission_created:
            self.current_state = States.AUTO
            print("STATE = %s" % self.current_state)
            self.stop_state = False
            self.next_state = States.HOVER
            print("Starting mission")
            # Reset mission set to first (0) waypoint
            self._vehicle.commands.next = 0

            # Set mode to AUTO to start mission
            # http://ardupilot.org/copter/docs/auto-mode.html
            self._vehicle.mode = VehicleMode("AUTO")

            # Monitor mission.
            # Demonstrates getting and setting the command number
            # Uses _distance_to_current_waypoint(), a convenience function for finding the
            #   distance to the next waypoint.

            while True:
                if not self.stop_state:
                    nextwaypoint = self._vehicle.commands.next
                    print("Distance to waypoint (%s): %sм" % (nextwaypoint, self._distance_to_current_waypoint))
                    time.sleep(1)
                else:
                    print("Interrupt state %s" % self.current_state + " switch to state %s" % self.next_state)
                    return self.next_state

            # print(
            #   "Successful completion of the state %s" % self.current_state + " switch to state %s" % self.next_state)
            # maybe you don't need stabilization, but just for everyone (suddenly failsafe will be)
            # self._need_hover = False
            # return self.next_state
        else:
            print(
                "Error: Create a mission beforehand! Now %s" % self.current_state + " switch to state %s" % self.next_state)
            return self.next_state


def cmd_handler(autocopter):
    """
    the command handler is executed in a separate thread,
    waits for user input and processes it
    """

    while True:
        CMD = input("Enter CMD: ")

        if autocopter.current_state == States.INIT:
            print("ERROR 2: COMMAND DENIED - INIT STATE - PLEASE WAIT")

        elif autocopter.current_state == States.IDLE:

            if CMD == "TAKEOFF":
                autocopter.new_state(States.TAKEOFF)
            elif CMD == "STATUS":
                print("\n%s" % autocopter.get_status)
            elif CMD == "LOCATION":
                lat, lon = autocopter.get_location()
                print(lat + ", " + lon)
            else:
                print("ERROR 3: CMD NOT APPLICABLE (TAKEOFF PENDING)")

        elif autocopter.current_state == States.TAKEOFF:

            if CMD == "LAND":
                autocopter.new_state(States.LAND)
            elif CMD == "HOVER":
                autocopter.new_state(States.HOVER)
            elif CMD == "STATUS":
                print("\n%s" % autocopter.get_status)
            elif CMD == "LOCATION":
                lat, lon = autocopter.get_location()
                print(lat + ", " + lon)
            else:
                print("ERROR 3: CMD NOT APPLICABLE (TAKEOFF IN PROCESS)")

        elif autocopter.current_state == States.HOVER:

            if CMD == "LAND":
                autocopter.new_state(States.LAND)
            elif CMD == "RTL":
                autocopter.new_state(States.RTL)
            elif CMD == "AUTO":
                autocopter.new_state(States.AUTO)
            elif CMD == "GOTO":
                WP = input("Specify WP:")

                if WP == "PTCL1":
                    autocopter.create_mission(coord_dict["PTCL1"][0], coord_dict["PTCL1"][1], coord_dict["PTCL1"][2])
                    autocopter.new_state(States.GOTO)
                if WP == "PTCL2":
                    autocopter.create_mission(coord_dict["PTCL2"][0], coord_dict["PTCL2"][1], coord_dict["PTCL2"][2])
                    autocopter.new_state(States.GOTO)
                if WP == "CASE1":
                    autocopter.create_mission(coord_dict["CASE1"][0], coord_dict["CASE1"][1], coord_dict["CASE1"][2])
                    autocopter.new_state(States.GOTO)
                if WP == "CASE2":
                    autocopter.create_mission(coord_dict["CASE2"][0], coord_dict["CASE2"][1], coord_dict["CASE2"][2])
                    autocopter.new_state(States.GOTO)
            elif CMD == "STATUS":
                print("\n%s" % autocopter.get_status)
            elif CMD == "LOCATION":
                lat, lon = autocopter.get_location()
                print(lat + ", " + lon)
            else:
                print("ERROR 3: CMD NOT APPLICABLE (HOVERING)")

        elif autocopter.current_state == States.RTL:

            if CMD == "LAND":
                autocopter.new_state(States.LAND)
            elif CMD == "HOVER":
                autocopter.new_state(States.HOVER)
            elif CMD == "STATUS":
                print("\n%s" % autocopter.get_status)
            elif CMD == "LOCATION":
                lat, lon = autocopter.get_location()
                print(lat + ", " + lon)
            else:
                print("ERROR 3: CMD NOT APPLICABLE (RTL IN PROCESS)")

        elif autocopter.current_state == States.LAND:

            if CMD == "HOVER":
                autocopter.new_state(States.HOVER)
            elif CMD == "STATUS":
                print("\n%s" % autocopter.get_status)
            elif CMD == "LOCATION":
                lat, lon = autocopter.get_location()
                print(lat + ", " + lon)
            else:
                print("ERROR 3: CMD NOT APPLICABLE (LANDING IN PROCESS)")

        elif autocopter.current_state == States.GOTO: #Dynamic Waypoint

            if CMD == "LAND":
                autocopter.new_state(States.LAND)
            elif CMD == "HOVER":
                autocopter.new_state(States.HOVER)
            elif CMD == "RTL":
                autocopter.new_state(States.RTL)
            elif CMD == "STATUS":
                print("\n%s" % autocopter.get_status)
            elif CMD == "LOCATION":
                lat, lon = autocopter.get_location()
                print(lat + ", " + lon)
            else:
                print("ERROR 3: CMD NOT APPLICABLE (GOTO IN PROCESS)")

        elif autocopter.current_state == States.AUTO:

            if CMD == "LAND":
                autocopter.new_state(States.LAND)
            elif CMD == "HOVER":
                autocopter.new_state(States.HOVER)
            elif CMD == "RTL":
                autocopter.new_state(States.RTL)
            elif CMD == "STATUS":
                print("\n%s" % autocopter.get_status)
            elif CMD == "LOCATION":
                lat, lon = autocopter.get_location()
                print(lat + ", " + lon)
            else:
                print("ERROR 3: CMD NOT APPLICABLE (AUTO MODE)")

        else:
            print("ERROR 1: INVALID COMMAND - NO HANDLER AVAILABLE")


def main():
    # CONNECTION_STR = "127.0.0.1:14561"
    autocopter = None
    try:
        autocopter = Autocopter(CONNECTION_STR)
        input_thread = threading.Thread(target=cmd_handler, args=(autocopter,))
        input_thread.start()

        while True:
            try:
                if autocopter.current_state == States.INIT:
                    autocopter.do_INIT()
                elif autocopter.current_state == States.IDLE:
                    autocopter.current_state = autocopter.do_IDLE()
                elif autocopter.current_state == States.TAKEOFF:
                    autocopter.current_state = autocopter.do_TAKEOFF()
                elif autocopter.current_state == States.HOVER:
                    autocopter.current_state = autocopter.do_HOVER()
                elif autocopter.current_state == States.RTL:
                    autocopter.current_state = autocopter.do_RTL()
                elif autocopter.current_state == States.AUTO:
                    autocopter.current_state = autocopter.do_AUTO()
                elif autocopter.current_state == States.LAND:
                    autocopter.current_state = autocopter.do_LAND()
                elif autocopter.current_state == States.GOTO:
                    autocopter.current_state = autocopter.do_GOTO()

            except Exception as ex:
                print("Error in state %s" % autocopter.current_state + ":\n" + str(
                    ex) + "\n" + traceback.format_exc() + "\n")
    except KeyboardInterrupt:
        pass
    finally:
        if autocopter is not None:
            autocopter.disconnect()


if __name__ == "__main__":
    CONNECTION_STR = "127.0.0.1:14561"
    main()
