from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from tools import get_location_metres, get_distance_metres, get_bearing
from pymavlink import mavutil
import numpy as np

connectionString = 'tcp:localhost:5762'
vehicle = connect(connectionString, wait_ready=True)

vehicle.mode = 'RTL'
vehicle.close()
