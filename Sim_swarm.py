import os
import sys
import threading
from time import sleep
import getpass

# N = int(sys.argv[1])
N = 2

k = 0
string = "mavproxy.py"

# def Launch_SITL_instances(i):
#     os.chdir("/home/" + getpass.getuser() + "/ardupilot/ArduCopter/")
#     os.system("sim_vehicle.py -I" + str(i) + " --out=127.0.0.1:" + str(14552 + i * 10) + " --sysid " + str(
#         i + 1) + " -L DTU" + str(i + 1))
#
#
# for i in range(N):
#     string += " --master=127.0.0.1:" + str(14551 + i * 10)
#     Launch_SITL_instances_thread = threading.Thread(target=Launch_SITL_instances, args=(i,))
#     Launch_SITL_instances_thread.start()
#     print("Done ")
#     sleep(4)
#
# os.system("gnome-terminal -e '" + string + " --map'")

# Creates a swarm of N drones having addresses on ports 14550, 14560, 14570...
for i in range(N):
    stringer = "sim_vehicle.py -v ArduCopter -I" + str(i) + " --out=127.0.0.1:" + str(
        14552 + i * 10) + " --out=127.0.0.1:" + str(14553 + i * 10) + " --sysid " + str(i + 1) + str(i + 1)
    os.system("gnome-terminal -- " + stringer)
    print("gnome-terminal -e " + stringer)
    string += " --master=127.0.0.1:" + str(14551 + i * 10)
    sleep(4)

os.system("gnome-terminal -e '" + string + " --map'")
