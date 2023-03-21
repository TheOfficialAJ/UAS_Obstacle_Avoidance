import sys

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from tools import get_location_metres, get_distance_metres, get_bearing
from pymavlink import mavutil
import numpy as np

connectionString = 'tcp:localhost:5762'
vehicle = connect(connectionString, wait_ready=True)

# K_ATT_X = 8 / 100
# K_ATT_Y = 8 / 100
# K_REP_X = 100
# K_REP_Y = 100

K_ATT_X = 7
K_ATT_Y = 7
K_REP_X = 7e2
K_REP_Y = 7e2
b = 100
a = 9.8e-2

home = LocationGlobalRelative(-35.3633512, 149.1652408)
print(home.lat, home.lon)
destination = get_location_metres(home, 150, 150)
obstacle1 = get_location_metres(home, 40, 50)
obstacle2 = LocationGlobalRelative(-35.3627775832047, 149.165770411491)
obstacle3 = LocationGlobalRelative(-35.3628169555, 149.1655583477)
print("Obstacle at: {};{}".format(obstacle1.lat, obstacle1.lon))
print("Bearing Between home and target: ", get_bearing(home, destination))
print("Destination at {};{}".format(destination.lat, destination.lon))


def arm_and_takeoff(aTargetAltitude):
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # FInal check to make sure vehicle is armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    if vehicle.location.global_relative_frame.alt < 1:
        vehicle.simple_takeoff(aTargetAltitude)
        while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)

            # Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)
    else:
        print("Already at target altitude")


# Function to calculate repulsive velocities due to obstacles
# TODO: Add obstacle avoidance
def get_repulsive_velocities(currentLocation, dest, trigger_radius, *obstacles):
    velocities = []
    safety_radius = 10  # Minimum radius to avoid collision
    obstacle_radius = 1
    for obstacle in obstacles:
        bearing = get_bearing(currentLocation, obstacle)
        print("Bearing to obstacle:", bearing)
        dist_obs = get_distance_metres(currentLocation, obstacle)
        currentLocation = vehicle.location.global_frame
        if dist_obs < obstacle_radius:
            print("##################-- COLLISION!!! --################", dist_obs)
            vehicle.mode = "RTL"
            vehicle.close()
            sys.exit()
        if dist_obs < trigger_radius:
            bearing = get_bearing(currentLocation, obstacle)
            dist_obs = get_distance_metres(currentLocation, obstacle)
            # rx = dist_obs * math.cos(np.radians(bearing))
            # ry = dist_obs * math.sin(np.radians(bearing))
            f_rep = -K_REP_Y * (1 / (dist_obs ** 2 - safety_radius ** 2) - 1 / (trigger_radius - safety_radius) ** 2)
            fx = f_rep * math.cos(np.radians(bearing))
            fy = f_rep * math.sin(np.radians(bearing))
            f_final_x = fx - fy  # Creates a rotational vector field in addition to the repulsive field to fix local
            f_final_y = fx + fy  # minima problems

            # vx = -K_REP_X * (1 / (rx ** 2 - 20 ** 2) - 1 / (safety_radius - 20) ** 2)
            # vy = -K_REP_Y * (1 / (ry ** 2 - 20 ** 2) - 1 / (safety_radius - 20) ** 2)
            velocities.append((f_final_x, f_final_y))
    return velocities


def apfa_navigate(home, dest, safety_radius, *obstacles):
    currentLocation = vehicle.location.global_frame
    dist_dest = get_distance_metres(currentLocation, dest)
    while dist_dest > 5:
        currentLocation = vehicle.location.global_frame
        print("Vehicle at {};{}".format(currentLocation.lat, currentLocation.lon))
        dist_dest = get_distance_metres(currentLocation, dest)
        print("Distance to destination:", dist_dest)
        bearing = get_bearing(currentLocation, dest)
        rx = dist_dest * math.cos(np.radians(bearing))
        ry = dist_dest * math.sin(np.radians(bearing))
        vx_att = (K_ATT_X * rx) / dist_dest
        vy_att = (K_ATT_Y * ry) / dist_dest
        vx_rep = 0
        vy_rep = 0
        rep_velocities = get_repulsive_velocities(currentLocation, dest, safety_radius, *obstacles)
        print("Repulsive Velocities", rep_velocities)
        for vel in rep_velocities:
            vx_rep += vel[0]
            vy_rep += vel[1]
        vx_tot = (vx_att + vx_rep)
        vy_tot = (vy_att + vy_rep)
        c = vx_tot / math.sqrt(vx_tot ** 2 + vy_tot ** 2)
        s = vy_tot / math.sqrt(vx_tot ** 2 + vy_tot ** 2)
        print("Velocity Angle:", np.degrees(np.arctan2(s, c)))
        print(c + s)
        vx_tot = 5 * c
        vy_tot = 5 * s
        # print("Bearing", bearing)
        print("Attraction velocity (x):", vx_att)
        print("Repulsive velocity (x):", vx_rep)
        print("Attraction velocity (y):", vy_att)
        print("Repulsive velocity (y):", vy_rep)
        print("Total velocity (x):", vx_tot)
        print("Total velocity (y):", vy_tot)

        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            vx_tot, vy_tot, 0,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # print("Sending velocity command: ", vx_tot, vy_tot)
        vehicle.send_mavlink(msg)
        # time.sleep(0.1)


takeoff_alt = 10.0
arm_and_takeoff(takeoff_alt)
home = vehicle.location.global_frame
print("Set default/target airspeed to 10")
# vehicle.airspeed = 10

print("Target location: ", "{};{}".format(destination.lat, destination.lon))
apfa_navigate(home, destination, 40, obstacle1, obstacle3)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

print("Closing vehicle object")
vehicle.close()
