import sys

from DroneNavigate import DroneNavigate
from dronekit import LocationGlobalRelative
from tools import *
from dronekit_sitl import SITL
# =========================----WAYPOINTS----=========================
home = LocationGlobalRelative(-35.3633512, 149.1652408, 0)
wp1 = LocationGlobalRelative(-35.362504163734, 149.16623711586, 10)
wp2 = LocationGlobalRelative(-35.362337924243, 149.16768014431, 20)
wp3 = LocationGlobalRelative(-35.3635672134375, 149.167063236237, 30)
# =========================----OBSTACLES----=========================
obstacle1 = LocationGlobalRelative(get_location_metres(home, 40, 50).lat, get_location_metres(home, 40, 50).lon, 8)
obstacle2 = LocationGlobalRelative(-35.3627775832047, 149.165770411491, 8)
obstacle3 = LocationGlobalRelative(-35.3628169555, 149.1655583477, 8)
obstacle4 = LocationGlobalRelative(-35.3623860462362, 149.16702568531, 22)
obstacle5 = LocationGlobalRelative(-35.3627972693709, 149.1677069664, 30)
obstacle6 = LocationGlobalRelative(-35.3628672645895, 149.167031049728, 30)

if __name__ == "__main__":
    try:
        # sitl = SITL()
        # sitl.download('copter', '3.3', verbose=True)
        # sitl.launch(['--model', 'quad', '--home=-35.3633512,149.1652408,584,353'], await_ready=True)

        drone = DroneNavigate()
        port = sys.argv[1]
        drone.connect('127.0.0.1:'+port)
        # drone.connect('tcp:127.0.0.1:5770')
        waypoints = [wp1, wp2, wp3]
        obstacles = [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6]
        drone.setHome(home)
        drone.setWaypoints(waypoints)
        drone.setSafetyRadius(20)
        drone.setObstacles(obstacles)
        drone.arm_and_takeoff(10)
        drone.navigate()
    except KeyboardInterrupt:
        print("KeyboardInterrupt: Returning to Launch")
        drone.setMode('RTL')
