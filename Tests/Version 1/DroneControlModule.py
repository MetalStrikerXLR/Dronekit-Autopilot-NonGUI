import math
import time
from dronekit import VehicleMode


def get_distance_metres(Location1, Location2):
    d_lat = Location2.lat - Location1.lat
    d_long = Location2.lon - Location1.lon
    return math.sqrt((d_lat * d_lat) + (d_long * d_long)) * 1.113195e5


def change_mode(drone, SpecifiedMode):
    drone.mode = VehicleMode(SpecifiedMode)


def arm(drone):
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not drone.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    while drone.location.global_relative_frame.alt is None:
        print(" Waiting for Parameters...")
        time.sleep(1)

    print("Arming Motors")
    # Copter should arm in GUIDED mode
    drone.mode = VehicleMode("GUIDED")
    drone.armed = True

    # Confirm vehicle armed before attempting to take off
    while not drone.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Drone Armed")


def disarm(drone):
    drone.armed = False
    # Confirm vehicle armed before attempting to take off
    while drone.armed:
        print(" Waiting for disarming...")
        time.sleep(1)

    print("Drone Disarmed")


def takeoff(drone, TargetAlt):
    print("Taking off!")
    drone.simple_takeoff(TargetAlt)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).

    while True:
        print(" Altitude: ", drone.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if drone.location.global_relative_frame.alt >= TargetAlt * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

    print("Takeoff Complete!")


def neutralize(drone, WP, target_distance, target_alt, AirSpd):
    drone.simple_goto(WP, airspeed=AirSpd)
    print("Going towards first point...")

    while True:
        current_alt = drone.location.global_relative_frame.alt
        current_distance = get_distance_metres(drone.location.global_frame, WP)
        print("Distance to WP: ", current_distance)
        print(" Altitude: ", current_alt)
        if current_alt >= target_alt * 0.95 and current_distance <= 0.1 * target_distance:
            print("Reached target")
            break
        time.sleep(1)


def exit_program(drone):
    # Close vehicle object before exiting script
    print("Close vehicle object")
    drone.close()
