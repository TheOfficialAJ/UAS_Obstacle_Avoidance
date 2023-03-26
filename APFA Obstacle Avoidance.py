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

K_ATT_X = 7.5
K_ATT_Y = 7.5
K_REP_X = 7e2
K_REP_Y = 7e2

K_VELOCITY_DEFAULT = 10
K_VELOCITY = K_VELOCITY_DEFAULT

home = LocationGlobalRelative(-35.3633512, 149.1652408)
print(home.lat, home.lon)
# wp1 = get_location_metres(home, 150, 150)

# ======================----WAYPOINTS----======================
wp1 = LocationGlobalRelative(-35.362504163734, 149.16623711586)
wp2 = LocationGlobalRelative(-35.362337924243, 149.16768014431)
wp3 = LocationGlobalRelative(-35.3635672134375, 149.167063236237)
# ======================----OBSTACLES----======================
obstacle1 = get_location_metres(home, 40, 50)
obstacle2 = LocationGlobalRelative(-35.3627775832047, 149.165770411491)
obstacle3 = LocationGlobalRelative(-35.3628169555, 149.1655583477)
obstacle4 = LocationGlobalRelative(-35.3623860462362, 149.16702568531)
obstacle5 = LocationGlobalRelative(-35.3627972693709, 149.1677069664)
obstacle6 = LocationGlobalRelative(-35.3628672645895, 149.167031049728)


# print("Obstacle at: {};{}".format(obstacle1.lat, obstacle1.lon))
# print("Bearing Between home and target: ", get_bearing(home, wp1))
# print("Destination at {};{}".format(wp1.lat, wp1.lon))


def sendVelocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        vx, vy, vz,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)


def arm_and_takeoff(aTargetAltitude):
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Final check to make sure vehicle is armed before attempting to take off
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


# TODO: Implement smooth braking to slow down drone when approaching obstacle
# Function to calculate repulsive velocities due to obstacles
def get_repulsive_velocities(currentLocation, r_wp_vec, trigger_radius, *obstacles):
    velocities = []
    safety_radius = 10  # Minimum radius to avoid collision
    obstacle_radius = 4
    for obstacle in obstacles:
        # bearing = get_bearing(currentLocation, obstacle)
        # print("Bearing to obstacle:", bearing)
        dist_obs = get_distance_metres(currentLocation, obstacle)
        currentLocation = vehicle.location.global_frame
        if dist_obs < obstacle_radius:
            print("##################-- COLLISION!!! --################", dist_obs)
            vehicle.mode = "RTL"
            vehicle.close()
            sys.exit()

        global K_VELOCITY
        if dist_obs < trigger_radius:
            if K_VELOCITY > 5:
                K_VELOCITY = 5
            bearing = get_bearing(currentLocation, obstacle)
            dist_obs = get_distance_metres(currentLocation, obstacle)
            bearing_home_obs = get_bearing(home, obstacle)
            dist_home_obs = get_distance_metres(home, obstacle)
            dist_home_obs_x = dist_home_obs * np.cos(math.radians(bearing_home_obs))
            dist_home_obs_y = dist_home_obs * np.sin(math.radians(bearing_home_obs))
            ro = np.array([dist_home_obs_x, dist_home_obs_y])
            # rx = dist_obs * math.cos(np.radians(bearing))
            # ry = dist_obs * math.sin(np.radians(bearing))
            # r_obs_vec = np.array([rx, ry])
            sign = np.sign(np.cross(r_wp_vec, ro))
            f_rep = -K_REP_Y * (1 / (dist_obs ** 2 - safety_radius ** 2) - 1 / (trigger_radius - safety_radius) ** 2)
            fx = f_rep * math.cos(np.radians(bearing))
            fy = f_rep * math.sin(np.radians(bearing))
            f_rot_x = -fy
            f_rot_y = fx
            print("Sign", sign)
            if not sign == 0:
                f_final_x = fx + sign * f_rot_x  # Creates a rotational vector field in addition to the repulsive field to fix local
                f_final_y = fy + sign * f_rot_y  # minima problem. We're adding a vortex vector field to the simple repulsive field
            else:
                f_final_x = fx + f_rot_x
                f_final_y = fy + f_rot_y
            # vx = -K_REP_X * (1 / (rx ** 2 - 20 ** 2) - 1 / (safety_radius - 20) ** 2)
            # vy = -K_REP_Y * (1 / (ry ** 2 - 20 ** 2) - 1 / (safety_radius - 20) ** 2)
            velocities.append((f_final_x, f_final_y))

    return velocities


def apfa_navigate(home, dest, safety_radius, *obstacles):
    global K_VELOCITY
    K_VELOCITY = K_VELOCITY_DEFAULT
    currentLocation = vehicle.location.global_frame
    dist_dest = get_distance_metres(currentLocation, dest)
    while dist_dest > 2:
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
        rep_velocities = get_repulsive_velocities(currentLocation, np.array([rx, ry]), safety_radius, *obstacles)
        print("Repulsive Velocities", rep_velocities)

        # If no obstacles nearby, then reset to max velocity
        if not rep_velocities:
            K_VELOCITY = K_VELOCITY_DEFAULT

        # If we are close to the waypoint, make velocities proportional to the distance (avoid overshoot)
        reduced_velocity = 1 + dist_dest/10
        if dist_dest < 100 and K_VELOCITY > reduced_velocity:
            K_VELOCITY = reduced_velocity
            print("CLOSE TO DESTINATION")

        for vel in rep_velocities:
            vx_rep += vel[0]
            vy_rep += vel[1]
        vx_tot = (vx_att + vx_rep)
        vy_tot = (vy_att + vy_rep)
        c = vx_tot / math.sqrt(vx_tot ** 2 + vy_tot ** 2)
        s = vy_tot / math.sqrt(vx_tot ** 2 + vy_tot ** 2)
        print("Velocity Angle:", np.degrees(np.arctan2(s, c)))
        print(c + s)
        vx_tot = K_VELOCITY * c
        vy_tot = K_VELOCITY * s
        # print("Bearing", bearing)
        print("Attraction velocity (x):", vx_att)
        print("Repulsive velocity (x):", vx_rep)
        print("Attraction velocity (y):", vy_att)
        print("Repulsive velocity (y):", vy_rep)
        print("Total velocity (x):", vx_tot)
        print("Total velocity (y):", vy_tot)

        sendVelocity(vx_tot, vy_tot, 0)
        # time.sleep(0.1)


try:
    takeoff_alt = 10.0
    arm_and_takeoff(takeoff_alt)
    home = vehicle.location.global_frame
    print("Set default/target airspeed to 10")
    # vehicle.airspeed = 10
    obstacle_list = [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6]
    print("Target location: ", "{};{}".format(wp1.lat, wp1.lon))
    apfa_navigate(home, wp1, 60, *obstacle_list)
    apfa_navigate(wp1, wp2, 60, *obstacle_list)
    apfa_navigate(wp2, wp3, 60, *obstacle_list)
    apfa_navigate(wp3, home, 60, *obstacle_list)

    sendVelocity(0, 0, 0)
    print("LANDING")
    vehicle.mode = VehicleMode("LAND")

    print("Closing vehicle object")
    vehicle.close()
except KeyboardInterrupt:  # In case of a keyboard interrupt, cause the vehicle to go into RTL mode and close the
    print("KeyboardInterrupt: Returning to Launch")  # vehicle object
    vehicle.mode = 'RTL'

    print("Closing vehicle object")
    vehicle.close()
