from time import sleep
from DroneNavigate import DroneNavigate
from dronekit import LocationGlobalRelative
from tools import *
import threading
from dronekit_sitl import SITL

# =========================----WAYPOINTS----=========================
home = LocationGlobalRelative(-35.3633512, 149.1652408, 10)
home_drone = LocationGlobalRelative(-35.36354998, 149.16508131, 10)
home_obstacle = LocationGlobalRelative(-35.36354516, 149.16729143, 10)
wp1 = LocationGlobalRelative(-35.362504163734, 149.16623711586, 10)
wp2 = LocationGlobalRelative(-35.362337924243, 149.16768014431, 20)
wp3 = LocationGlobalRelative(-35.3635672134375, 149.167063236237, 30)
# =========================----OBSTACLES----=========================
obstacle1 = LocationGlobalRelative(get_location_metres(home, 40, 50).lat, get_location_metres(home, 40, 50).lon, 8)
wp_drone = LocationGlobalRelative(-35.36174761, 149.16728552, 10)
wp_obstacle = LocationGlobalRelative(-35.36176207, 149.16509312, 10)
obstacle2 = LocationGlobalRelative(-35.3627775832047, 149.165770411491, 8)
obstacle3 = LocationGlobalRelative(-35.3628169555, 149.1655583477, 8)
obstacle4 = LocationGlobalRelative(-35.3623860462362, 149.16702568531, 22)
obstacle5 = LocationGlobalRelative(-35.3627972693709, 149.1677069664, 30)
obstacle6 = LocationGlobalRelative(-35.3628672645895, 149.167031049728, 30)

obs_start_loc = LocationGlobalRelative(-35.36330283, 149.16671747, 10)
obs_end_loc = LocationGlobalRelative(-35.36185134, 149.16512415, 10)

obstacleDrone = None


def startObstacle(connectionString):
    global obstacleDrone
    obstacleDrone = DroneNavigate()
    obstacleDrone.connect(connectionString)
    obstacleDrone.arm_and_takeoff(10)
    while True:
        while get_distance_metres(obstacleDrone.getCurrentLocation(), home_obstacle) > 2:
            obstacleDrone.goto(home_obstacle)
        while get_distance_metres(obstacleDrone.getCurrentLocation(), wp_obstacle) > 2:
            obstacleDrone.goto(wp_obstacle)


def startDrone():
    try:
        # drone = DroneNavigate()
        # port = sys.argv[1]
        # drone.connect('127.0.0.1:' + str(port))
        # # drone.connect('tcp:127.0.0.1:5770')
        # waypoints = [wp1, wp2, wp3]
        # obstacles = [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6]
        # drone.setHome(home)
        # drone.setWaypoints(waypoints)
        # drone.setSafetyRadius(20)
        # drone.setObstacles(obstacles)
        # drone.arm_and_takeoff(10)
        # drone.navigate()
        drone = DroneNavigate()
        drone.connect("127.0.0.1:14550")
        print(obstacleDrone)
        drone.setNeighbours([obstacleDrone])
        drone.setHome(home_drone)
        drone.setWaypoints([wp_drone])
        drone.arm_and_takeoff(10)
        while True:
            drone.navigate()
            drone.setWaypoints([home_drone])
            drone.navigate()
            drone.setWaypoints([wp_drone])

    except KeyboardInterrupt:
        print("KeyboardInterrupt: Returning to Launch")
        drone.setMode('RTL')


if __name__ == "__main__":
    obstacle_thread = threading.Thread(target=startObstacle, args=("127.0.0.1:14560",))
    drone_thread = threading.Thread(target=startDrone)

    obstacle_thread.start()
    drone_thread.start()
