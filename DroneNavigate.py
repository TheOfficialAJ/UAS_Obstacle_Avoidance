import sys

import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from tools import *
from pymavlink import mavutil
import numpy as np
import itertools


class DroneNavigate:
    """
    This is a class used to navigate a drone using 3D Artificial Potential Field Algorithm (APFA) through a set of
    waypoints avoiding obstacles.
    """
    K_ATT_X = 7.3
    K_ATT_Y = 7.3
    K_ATT_Z = 7.6  # Having a higher K_ATT_Z reduces the chances of your drone crashing into the ground :)
    K_REP_X = 7e2
    K_REP_Y = 7e2
    K_VELOCITY_DEFAULT = 10
    K_VELOCITY = K_VELOCITY_DEFAULT

    def __init__(self):
        """
        Initializes all the class variables
        """

        self.connectionString = None
        self.vehicle = None
        self.home = None
        self.waypoints = []
        self.currentWaypoint = None
        self.static_obstacle_locations = []
        self.dynamic_obstacle_locations = []
        self.neighbours = []
        self.safety_radius = 70
        self.obstacle_radius = 5
        self.trigger_radius = 100

    def setNeighbours(self, neighbours):
        """
        Sets the neighbours of the UAV, neighbours are other DroneNavigate objects that are in the vicinity of the UAV
        :param neighbours: A list of DroneNavigate objects
        :return: None
        """
        self.neighbours = neighbours

    def getCurrentLocation(self):
        return self.vehicle.location.global_relative_frame

    def connect(self, connectionString):
        """
        Connects to the drone and intializes the vehicle object
        :param connectionString: The connection string specifying the connection address
        :return: None
        """
        self.vehicle = connect(connectionString, wait_ready=False)

    def setMode(self, mode):
        self.vehicle.mode = mode

    def setSafetyRadius(self, safety_radius):
        self.safety_radius = safety_radius

    def navigate(self):
        for wp in self.waypoints:
            self.currentWaypoint = wp
            print(wp)
            self._apfa_navigate(self.currentWaypoint)

    def arm_and_takeoff(self, aTargetAltitude):
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Final check to make sure vehicle is armed before attempting to take off
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        if self.vehicle.location.global_relative_frame.alt < 1:
            self.vehicle.simple_takeoff(aTargetAltitude)
            while True:
                print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)

                # Break and return from function just below target altitude.
                if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                    print("Reached target altitude")
                    break
                time.sleep(1)
        else:
            print("Already at target altitude")

    def sendVelocity(self, vx, vy, vz):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            vx, vy, vz,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(msg)

    def goto(self, location):
        self.vehicle.groundspeed = 10
        self.vehicle.simple_goto(location)

    def setHome(self, home):
        """
        Sets the home location of the drone
        :param home: A LocationGlobal or LocationGlobalRelative object
        :return: None
        """
        self.home = home

    def setWaypoints(self, waypoints):
        """
        Intitializes the waypoints list, the drone navigates the waypoints sequentially in the order they are added
        :param waypoints: A list of LocationGlobal or LocationGlobalRelative objects
        :return: None
        """
        self.waypoints = waypoints

    def setObstacles(self, obstacles):
        """
        Initializes the obstacles list, the drone avoids the obstacles while navigating the waypoints
        :param obstacles: A list of LocationGlobal or LocationGlobalRelative objects
        :return: None
        """
        self.static_obstacle_locations = obstacles

    def _apfa_navigate(self, dest):
        self.K_VELOCITY = self.K_VELOCITY_DEFAULT
        currentLocation = self.vehicle.location.global_relative_frame
        dist_dest = get_distance_metres(currentLocation, dest)
        while dist_dest > 2:
            print("Vehicle at {};{}".format(currentLocation.lat, currentLocation.lon))

            # TODO: Updates the current location and the location of dynamic obstacles (other UAVs)

            currentLocation = self.vehicle.location.global_relative_frame

            dist_2d_dest = get_distance_metres(currentLocation, dest)
            dist_dest = get_3d_distance(currentLocation, dest)
            alt_change = dest.alt - currentLocation.alt
            print("Distance to destination:", dist_dest)
            bearing = get_bearing(currentLocation, dest)
            phi = math.atan2(alt_change, dist_2d_dest)
            rx = dist_dest * math.cos(np.radians(bearing)) * math.cos(phi)
            ry = dist_dest * math.sin(np.radians(bearing)) * math.cos(phi)
            rz = dist_dest * math.sin(phi)
            vx_att = (self.K_ATT_X * rx) / dist_dest
            vy_att = (self.K_ATT_Y * ry) / dist_dest
            vz_att = (self.K_ATT_Z * rz) / dist_dest

            # ----------------Calculating repulsive velocities----------------
            vx_rep = 0
            vy_rep = 0
            vz_rep = 0
            rep_velocities = self._get_repulsive_velocities(np.array([rx, ry]), 100)
            print("Repulsive Velocities", rep_velocities)

            # If no obstacles nearby, then reset to max velocity
            if not rep_velocities:
                self.K_VELOCITY = self.K_VELOCITY_DEFAULT

            # If we are close to the waypoint, make velocities proportional to the distance (avoid overshoot)
            reduced_velocity = 2 + dist_dest / 10
            if dist_dest < 100 and self.K_VELOCITY > reduced_velocity:
                self.K_VELOCITY = reduced_velocity
                print("CLOSE TO DESTINATION")

            for vel in rep_velocities:
                vx_rep += vel[0]
                vy_rep += vel[1]
                vz_rep += vel[2]
            vx_tot = (vx_att + vx_rep)
            vy_tot = (vy_att + vy_rep)
            vz_tot = (vz_att + vy_rep)
            print(vz_tot)
            fin_vel_x = vx_tot / math.sqrt(vx_tot ** 2 + vy_tot ** 2 + vz_tot ** 2)
            fin_vel_y = vy_tot / math.sqrt(vx_tot ** 2 + vy_tot ** 2 + vz_tot ** 2)
            fin_vel_z = vz_tot / math.sqrt(vx_tot ** 2 + vy_tot ** 2 + vz_tot ** 2)
            print("Velocity Angle:", np.degrees(np.arctan2(fin_vel_y, fin_vel_x)))
            print(fin_vel_x + fin_vel_y)
            vx_tot = self.K_VELOCITY * fin_vel_x
            vy_tot = self.K_VELOCITY * fin_vel_y
            vz_tot = self.K_VELOCITY * fin_vel_z
            # print("Bearing", bearing)
            print("Attraction velocity (x):", vx_att)
            print("Repulsive velocity (x):", vx_rep)
            print("Attraction velocity (y):", vy_att)
            print("Repulsive velocity (y):", vy_rep)
            print("Total velocity (x):", vx_tot)
            print("Total velocity (y):", vy_tot)
            print("Total velocity (z):", vz_tot)
            self.sendVelocity(vx_tot, vy_tot, -vz_tot)
            # time.sleep(0.1)

    def update_dynamic_obstacles(self):
        for neighbour in self.neighbours:
            self.dynamic_obstacle_locations = []
            self.dynamic_obstacle_locations.append(neighbour.getCurrentLocation())

    # TODO: Implement smooth braking to slow down drone when approaching obstacle
    # Function to calculate repulsive velocities due to obstacles
    def _get_repulsive_velocities(self, r_wp_vec, trigger_radius):
        velocities = []
        for obstacle in self.static_obstacle_locations:
            # bearing = get_bearing(currentLocation, obstacle)
            # print("Bearing to obstacle:", bearing)
            currentLocation = self.getCurrentLocation()
            dist_obs = get_3d_distance(currentLocation, obstacle)
            print("Distance to obstacle:", dist_obs)
            if dist_obs < self.obstacle_radius:
                print("##################-- COLLISION!!! --################", dist_obs)
                self.vehicle.mode = "RTL"
                self.vehicle.close()
                sys.exit()

            if dist_obs < self.trigger_radius:
                # if self.K_VELOCITY > 5:
                #     self.K_VELOCITY = 5
                f_final_x, f_final_y, f_final_z = self._calc_repulsive_velocity(currentLocation, obstacle, r_wp_vec)
                velocities.append((f_final_x, f_final_y, f_final_z))

        self.update_dynamic_obstacles()
        for obstacle in self.dynamic_obstacle_locations:
            currentLocation = self.getCurrentLocation()
            dist_obs = get_3d_distance(currentLocation, obstacle)
            print("Distance to obstacle:", dist_obs)
            if dist_obs < self.obstacle_radius:
                print("##################-- COLLISION!!! --################", dist_obs)
                self.vehicle.mode = "RTL"
                self.vehicle.close()
                sys.exit()

            if dist_obs < self.trigger_radius:
                # if self.K_VELOCITY > 5:
                #     self.K_VELOCITY = 5
                f_final_x, f_final_y, f_final_z = self._calc_repulsive_velocity(currentLocation, obstacle, r_wp_vec)
                velocities.append((f_final_x, f_final_y, f_final_z))
        return velocities

    def _calc_repulsive_velocity(self, currentLocation, obstacle, r_wp_vec):
        bearing = get_bearing(currentLocation, obstacle)
        dist_obs = get_3d_distance(currentLocation, obstacle)
        dist_2d = get_distance_metres(currentLocation, obstacle)
        alt_change = obstacle.alt - currentLocation.alt
        phi = math.atan2(alt_change, dist_2d)
        bearing_home_obs = get_bearing(self.currentWaypoint, obstacle)
        dist_home_obs = get_3d_distance(self.currentWaypoint, obstacle)
        dist_home_obs_x = dist_home_obs * np.cos(math.radians(bearing_home_obs)) * np.cos(phi)
        dist_home_obs_y = dist_home_obs * np.sin(math.radians(bearing_home_obs)) + np.cos(phi)
        ro = np.array([dist_home_obs_x, dist_home_obs_y])
        sign = np.sign(np.cross(r_wp_vec, ro))
        f_rep = -self.K_REP_Y * (
                1 / (dist_obs ** 2 - self.safety_radius ** 2) - 1 / (self.trigger_radius - self.safety_radius) ** 2)
        fx = f_rep * math.cos(np.radians(bearing)) * np.cos(phi)
        fy = f_rep * math.sin(np.radians(bearing)) * np.cos(phi)
        fz = f_rep * np.sin(phi)
        f_rot_x = -fy
        f_rot_y = fx
        print("Sign", sign)
        if not sign == 0:
            f_final_x = fx + sign * f_rot_x  # Creates a rotational vector field in addition to the repulsive field to fix local
            f_final_y = fy + sign * f_rot_y  # minima problem. We're adding a vortex vector field to the simple repulsive field
        else:
            f_final_x = fx + f_rot_x
            f_final_y = fy + f_rot_y
        f_final_z = fz
        return f_final_x, f_final_y, f_final_z
# wp1 = get_location_metres(home, 150, 150)


# print("Obstacle at: {};{}".format(obstacle1.lat, obstacle1.lon))
# print("Bearing Between home and target: ", get_bearing(home, wp1))
# print("Destination at {};{}".format(wp1.lat, wp1.lon))
