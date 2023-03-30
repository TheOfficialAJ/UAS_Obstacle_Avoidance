from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from tools import get_location_metres, get_distance_metres, get_bearing
from pymavlink import mavutil
import numpy as np

connectionString = '127.0.0.1:14550'
vehicle = connect(connectionString, wait_ready=False)
vehicle.mode = 'RTL'
vehicle.close()
