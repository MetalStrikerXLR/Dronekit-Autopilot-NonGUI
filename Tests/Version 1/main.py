from __future__ import print_function
import threading
import DroneControlModule as DCM
from dronekit import connect, LocationGlobalRelative

coord_dict = {
    "PTCL1": [float(33.6730629), float(73.0523336), float(20)],
    "PTCL2": [float(33.6729402), float(73.0520667), float(5)],
    "CASE1": [float(33.6940573), float(72.8235720), float(30)],
    "CASE2": [float(33.6938386), float(72.8231174), float(5)]
}


# Input Thread
def input_thread():
    while True:
        cmd = input("Enter Command (arm,takeoff,rtl,land,guided,loiter,auto):")

        if cmd == "arm" or cmd == "ARM":
            print("Here 1")
            DCM.arm(drone)

        if cmd == "disarm" or cmd == "DISARM":
            DCM.disarm(drone)

        if cmd == "takeoff" or cmd == "TAKEOFF":
            target_alt = input("Specify Target Altitude (meters): ")

            takeoff_thread = threading.Thread(target=DCM.takeoff, args=(drone, int(target_alt)))
            takeoff_thread.start()

        if cmd == "neut" or cmd == "NEUT":
            wp = LocationGlobalRelative(coord_dict["PTCL1"][0], coord_dict["PTCL1"][1], coord_dict["PTCL1"][2])
            wp_distance = DCM.get_distance_metres(drone.location.global_frame, wp)
            wp_altitude = coord_dict["PTCL1"][2]
            airspeed = 10

            neut_thread = threading.Thread(target=DCM.neutralize, args=(drone, wp, wp_distance, wp_altitude, airspeed))
            neut_thread.start()

        if cmd == "rtl" or cmd == "RTL":
            DCM.change_mode(drone, "RTL")

        if cmd == "guided" or cmd == "GUIDED":
            DCM.change_mode(drone, "GUIDED")

        if cmd == "auto" or cmd == "AUTO":
            DCM.change_mode(drone, "AUTO")

        if cmd == "loiter" or cmd == "LOITER":
            DCM.change_mode(drone, "LOITER")

        if cmd == "stabilize" or cmd == "STABILIZE":
            DCM.change_mode(drone, "STABILIZE")

        if cmd == "land" or cmd == "LAND":
            DCM.change_mode(drone, "LAND")

        if cmd == "exit" or cmd == "EXIT":
            DCM.exit_program(drone)
            print("Program Exited...")
            break


# Main Code
print('Connecting to Drone...')
drone = connect('127.0.0.1:14561', wait_ready=False)
print("Connection Successful!")
print("")

# Start Threads
thread1 = threading.Thread(target=input_thread, args=())
thread1.start()
