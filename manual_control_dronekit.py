"""
Example of how to send MANUAL_CONTROL messages to the autopilot using
pymavlink.
This message is able to fully replace the joystick inputs.
"""

# Import mavutil
from pymavlink import mavutil
import time

# doesnt work with px4 sitl

# Create the connection
print("connecting")
master = mavutil.mavlink_connection('udpin:127.0.0.1:14540') #'14550') # "udp://:14540"
# Wait a heartbeat before sending commands
print("connected..")
#master.wait_heartbeat()
#print("connected.. got heartbeat")
# Send a positive x value, negative y, negative z,
# positive rotation and no button.
# https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
# Warning: Because of some legacy workaround, z will work between [0-1000]
# where 0 is full reverse, 500 is no output and 1000 is full throttle.
# x,y and r will be between [-1000 and 1000].
master.mav.manual_control_send(
    master.target_system,
    0,
    0,
    0,
    500,
    0)

# To active button 0 (first button), 3 (fourth button) and 7 (eighth button)
# It's possible to check and configure this buttons in the Joystick menu of QGC

# doesnt work with px4 sitl
while(master): 
    print("looping...sending yaw")
    buttons = 1 + 1 << 3 + 1 << 7
    master.mav.manual_control_send(
    master.target_system,
    0,
    0,
    500, # 500 means neutral throttle
    -1000,
    buttons)
    time.sleep(0.1)
