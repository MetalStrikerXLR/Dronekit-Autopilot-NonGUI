from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative


# Connect to the Vehicle
print('Connecting to Drone...')
drone = connect('127.0.0.1:14551', wait_ready=False, baud=57600)

# Main Code
drone.mode = VehicleMode("GUIDED")
print("Vehicle set to GUIDED Mode")

print("Setting airspeed to 5m/s")
drone.airspeed = 5    # Set Vehicle Speed

# Coordinate 1
# lat1 = float(33.6730953)
# long1 = float(73.0522209)
# alt1 = float(30)

lat1 = float(33.6940573)
long1 = float(72.8235720)
alt1 = float(30)

point1 = LocationGlobalRelative(lat1, long1, alt1)
drone.simple_goto(point1, airspeed=10)
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

# Close vehicle object before exiting script
print("Close vehicle object")
drone.close()