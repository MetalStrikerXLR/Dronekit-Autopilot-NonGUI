from __future__ import print_function
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative


# Connect to the Vehicle
print('Connecting to Drone...')
drone = connect('127.0.0.1:14551', wait_ready=False)


def get_distance_metres(Location1, Location2):
    dlat = Location2.lat - Location1.lat
    dlong = Location2.lon - Location1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not drone.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    while drone.location.global_relative_frame.alt == None:
        print(" Waiting for Parameters...")
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


# Main Code
arm_and_takeoff(10)    # Input Altitude in meters

print("Setting airspeed to 5m/s")
drone.airspeed = 5    # Set Vehicle Speed

# Coordinate 1
lat1 = float(33.6730629)      # PTCL P1
long1 = float(73.0523336)
target_alt1 = float(20)


# lat1 = float(33.6940573)        # CASE 2
# long1 = float(72.8235720)
# alt1 = float(30)

point1 = LocationGlobalRelative(lat1, long1, target_alt1)
target_dist1 = get_distance_metres(drone.location.global_frame, point1)
drone.simple_goto(point1, airspeed=10)
print("Going towards first point...")

while True:
    current_alt = drone.location.global_relative_frame.alt
    current_distance = get_distance_metres(drone.location.global_frame, point1)
    print("Distance to WP: ", current_distance)
    print(" Altitude: ", current_alt)
    if current_alt >= target_alt1 * 0.95 and current_distance <= 0.1 * target_dist1:
        print("Reached target")
        break
    time.sleep(1)


# Coordinate 2
lat2 = float(33.6729402)      # PTCL P2
long2 = float(73.0520667)
target_alt2 = float(5)

# lat2 = float(33.6938386)        # CASE-P2
# long2 = float(72.8231174)
# alt2 = float(5)

point2 = LocationGlobalRelative(lat2, long2, target_alt2)
target_dist2 = get_distance_metres(drone.location.global_frame, point2)
drone.simple_goto(point2, groundspeed=10)
print("Going towards second point for 30 seconds ...")

while True:
    current_alt = drone.location.global_relative_frame.alt
    current_distance = get_distance_metres(drone.location.global_frame, point2)
    print("Distance to WP: ", current_distance)
    print(" Altitude: ", current_alt)
    if current_alt >= target_alt2 * 0.95 and current_distance <= 0.1 * target_dist2:
        print("Reached target")
        break
    time.sleep(1)


# Mission Complete, Return to Launch
print("Returning to Launch")
drone.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
drone.close()