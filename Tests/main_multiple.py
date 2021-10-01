from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative


# Connect to the Vehicle
print('Connecting to Drone...')
drone = connect('127.0.0.1:14551', wait_ready=False, baud=57600)
drone2 = connect('127.0.0.1:14561', wait_ready=False, baud=57600)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not drone.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    drone.mode = VehicleMode("GUIDED")
    drone.armed = True

    # Confirm vehicle armed before attempting to take off
    while not drone.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    drone.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", drone.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if drone.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def arm_and_takeoff2(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not drone2.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    drone2.mode = VehicleMode("GUIDED")
    drone2.armed = True

    # Confirm vehicle armed before attempting to take off
    while not drone2.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    drone2.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", drone2.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if drone2.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


# Main Code
arm_and_takeoff(10)     # Input Altitude in meters
arm_and_takeoff2(10)     # Input Altitude in meters

# drone.mode = VehicleMode("GUIDED")
# print("Vehicle set to GUIDED Mode")

print("Setting airspeed to 5m/s")
drone.airspeed = 5    # Set Vehicle Speed
drone2.airspeed = 20

# Coordinate 1
dlat1 = float(33.6730953)
dlong1 = float(73.0522209)
dalt1 = float(30)

lat1 = float(33.6940573)
long1 = float(72.8235720)
alt1 = float(30)

point1 = LocationGlobalRelative(lat1, long1, alt1)
dpoint1 = LocationGlobalRelative(dlat1, dlong1, dalt1)
drone.simple_goto(point1, airspeed=10)
drone2.simple_goto(dpoint1, airspeed=10)
print("Going towards first point for 30 seconds ...")

while True:
    print(" Altitude: ", drone.location.global_relative_frame.alt)
    if drone.location.global_relative_frame.alt >= alt1 * 0.95:
        print("Reached target")
        break
    time.sleep(1)


# Coordinate 2
# lat2 = float(33.6728721)
# long2 = float(73.0517569)
# alt2 = float(5)

lat2 = float(33.6938386)
long2 = float(72.8231174)
alt2 = float(5)
point2 = LocationGlobalRelative(lat2, long2, alt2)
drone.simple_goto(point2, groundspeed=10)
print("Going towards second point for 30 seconds ...")

while True:
    print(" Altitude: ", drone.location.global_relative_frame.alt)
    if drone.location.global_relative_frame.alt <= alt2 * 1.05:
        print("Reached target altitude")
        break
    time.sleep(1)


# Mission Complete, Return to Launch
print("Returning to Launch")
drone.mode = VehicleMode("RTL")
drone2.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
drone.close()
drone2.close()