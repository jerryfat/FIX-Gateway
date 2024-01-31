#!/usr/bin/env python3
from platform import python_version
print("using Python ver:",python_version())
#
# for mavsdk
#import asyncio
#from mavsdk import start_mavlink
#from mavsdk import System



from math import *

from datetime import datetime, timedelta

import time, sys, argparse
import socket
import pickle
import subprocess 
import json
import re

#
from reprint import output
#
################################################################################################
# Settings
################################################################################################

Px4simAddrPort="" # connection string incoming mavlink msgs from px4 ardupilot oreal or sitl
pyG5SimAddrPort=""
pyG5SimAddr = ""
pyG5SimPort = 0
pyefisSimAddrPort=""
pyefisSimAddr = ""
pyefisSimPort = 0

useMAVSDK = False
useDronekit = False
# timestamp date_time
now = datetime.now()
date_time = now.strftime("%m/%d/%Y, %H:%M:%S")

MAV_MODE_AUTO   = 4
data_dict = {}

# timestamp date_time
# fixgw $ python3 ./fixgw.py -v -d -config-file "fixgw/configs/default.yaml"
# sitl #$ sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2  
#docker run --rm -it jonasvautherin/px4-gazebo-headless:1.14.0
#$ sudo docker run --rm -it --env PX4_HOME_LAT=42.397742 --env PX4_HOME_LON=-79.1 --env PX4_HOME_ALT=488.0 jonasvautherin/px4-gazebo-headless:1.14.0
# python mavlinkMAVSDKdronekitCombined.py -m "127.0.0.1:14445" -g 127.0.0.1:65432 -e 127.0.0.1:2100 # px4 sitl using docker and qgcs forwarding on 14445
#https://github.com/JonasVautherin/px4-gazebo-headless
# USEAGE
#  MAVSDK connect strings, use :// to automagically use MAVSDK libs below
# $ python mavlinkMAVSDKdronekitCombined.py -m "udp://:14540" -g 127.0.0.1:65432 -e 127.0.0.1:2100 #works
# $ python mavlinkMAVSDKdronekitCombined.py  -m "serial:///dev/ttyUSB0:57600" -g 127.0.0.1:65432 -e 127.0.0.1:2100
# $ python mavlinkMAVSDKdronekitCombined.py  -m "serial:///dev/ttyUSB0:57600" -g 127.0.0.1:65432 -e 127.0.0.1:2100
# $ python3 FIX-Gateway/mavlinkMAVSDKdronekitCombined.py -m "/dev/ttyUSB0,57600" -g 127.0.0.1:65432 -e 127.0.0.1:2100
# $ python3 FIX-Gateway/mavlinkMAVSDKdronekitCombined.py -m "udp://172.17.0.1:14540" -g 127.0.0.1:65432 -e 127.0.0.1:2100 ?
#
# dronekit connect strings 
#python3 FIX-Gateway/mavlinkMAVSDKdronekitCombined.py -m "172.17.0.1:14540" -g 127.0.0.1:65432 -e 127.0.0.1:2100
#python mavlinkMAVSDKdronekitCombined.py  -m 172.17.0.1:14540 -g 127.0.0.1:65432 -e 127.0.0.1:2100
#python3 ./mavlinkMAVSDKdronekitCombined.py -m /dev/ttyUSB0,57600 -g 127.0.0.1:65432 -e 127.0.0.1:2100 pixhawk usb  
#python3 ./mavlinkMAVSDKdronekitCombined.py -m /dev/ttyACM0,57600 -g 127.0.0.1:65432 -e 127.0.0.1:2100  sik radio  
#python3 ./mavlinkMAVSDKdronekitCombined.py -m 172.17.0.1:14550 -g 127.0.0.1:65432 -e 127.0.0.1:2100 sitl  
#python3 ./mavlinkMAVSDKdronekitCombined.py -m 127.0.0.1:14445 -g 127.0.0.1:65432 -e 127.0.0.1:2100 qgcs forwarded  
#to start sitl px4 gazebo sim locally via docker app:  

# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py
# https://firmware.ardupilot.org/Plane/latest/Pixhawk1-1M/  ARDUPLANE #4.5.0-FIRMWARE_VERSION_TYPE_DEV
# https://github.com/ArduPilot/pymavlink/blob/master/mavextra.py
# USEAGE
# $ python telemetry.py -m "serial:///dev/ttyUSB0:57600" -g 127.0.0.1:65432 -e 127.0.0.1:2100
# $ python3 FIX-Gateway/mavlink2PX4G5.py -m "/dev/ttyUSB0,57600" -g 127.0.0.1:65432 -e 127.0.0.1:2100
# $ python telemetry.py -m "udp://:14540" -g 127.0.0.1:65432 -e 127.0.0.1:2100
#$ python telemetry.py -m "udp://:14540" -g 127.0.0.1:65432 -e 127.0.0.1:2100
#$ python telemetry.py -m ":14445" -g 127.0.0.1:65432 -e 127.0.0.1:2100
#$ python telemetry.py -m "udp://:14445" -g 127.0.0.1:65432 -e 127.0.0.1:2100
#python3 ./mavlink2pyg5.py -m /dev/ttyUSB0,57600 -g 127.0.0.1:65432 -e 127.0.0.1:2100 pixhawk usb  
#python3 ./mavlink2pyg5.py -m /dev/ttyACM0,57600 -g 127.0.0.1:65432 -e 127.0.0.1:2100  sik radio  
#python3 ./mavlink2pyg5.py -m 172.17.0.1:14550 -g 127.0.0.1:65432 -e 127.0.0.1:2100 sitl  
#python3 ./mavlink2PX4G5.py -m 127.0.0.1:14445 -g 127.0.0.1:65432 -e 127.0.0.1:2100 qgcs forwarded  
#to starrt sitl px4 gazebo sim locally via docker app:  
#$ sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2  

################################################################################################
################################################################################################
################################################################################################
# Parse connection argument
parser = argparse.ArgumentParser()
#parser.add_argument("-c", "--connect", help="connection string")
parser.add_argument("-m", "--px4", help="recv MAV from mavlink dronekit connection string /dev/ttyUSB0,57600, /dev/ttyACM0,57600 or sitl 172.17.0.1:14550, qgcs 127.0.0.1:14445 for MAVSDK connect -m serial:///dev/ttyUSB0:57600 or -m udp://:14540")
parser.add_argument("-g", "--pyg5", help="pyg5sim sendto address:port 127.0.0.1:65432 same as xplane for now")
parser.add_argument("-e", "--pyefis", help="pyefis sendto address:port 127.0.0.1:2100")
args = parser.parse_args() # command line parameters
# PX4 input mavlink messages
if args.px4: # input from sitl qgcs or usb serial port to autopilot
    #connection_string = args.px4
    Px4simAddrPort = args.px4 # for MAVSDK
    connection_string = args.px4 # for dronekit
    print("Px4simAddrPort -", Px4simAddrPort)
    if ("://" in Px4simAddrPort):
        useMAVSDK = True
        useDronekit = False
        print("#### command line option detected to use MAVSDK for connect, does have :// in connect string , useMAVSDK, useDronekit", useMAVSDK, useDronekit)
    else:
        useMAVSDK = False
        useDronekit = True
        print("#### command line option detected to use Dronekit connect, has no :// in connect string , useMAVSDK, useDronekit", useMAVSDK, useDronekit)
# pyG5 output to pyG5
if args.pyg5:
    pyG5SimAddrPort = args.pyg5 #pyG5SimAddrPort = "127.0.0.1:65432"  # reuse xplane port for now
    pyG5str = pyG5SimAddrPort.split(":")  #"127.0.0.1"  # The server's hostname or IP address
    pyG5SimAddr = pyG5str[0] # 127.0.0.1
    pyG5SimPort = int(pyG5str[1]) # 2100 #65432  # The port used by the server
    print("pyG5SimAddr -",pyG5SimAddr, "  pyG5SimPort-",pyG5SimPort)
# pyEfis output to Efis
if args.pyefis:
    pyefisSimAddrPort = args.pyefis
    pyefisSimStr = pyefisSimAddrPort.split(":")  #"127.0.0.1"  # The server's hostname or IP address
    pyefisSimAddr = pyefisSimStr[0] # 127.0.0.1
    pyefisSimPort = int(pyefisSimStr[1]) # 2100 #65432  # The port used by the server
    print("pyefisSimAddr-",pyefisSimAddr,"  pyefisSimPort-",pyefisSimPort)


print("parsed args: Px4simAddrPort-", Px4simAddrPort, "  pyG5SimAddrPort-",pyG5SimAddrPort,"  pyefisSimAddrPort-",pyefisSimAddrPort)
#### END ARG PARSER ################################################################################################



def SendAttitudeDataToG5simEfisSimDronekit(msg): # format and load as dict and then pickle and send to pyG5 or pyEfis
    global data_dict
    print("--DRONEKIT--Recvd msgId:", msg.get_msgId(), "msg:", msg )
    tmp = str(msg).split(" ") #print("tmp[1:]=", tmp[1:])
    text = re.sub('(\w+)\s?:\s?("?[^",]+"?,?)', "\"\g<1>\":\g<2>", str(tmp[1:])) #print("text ",text)
    tmp2= str(tmp[1:]).replace("'{","'") #print("tmp2=", tmp2)
    tmp3= tmp2.replace("}'","'") #print("tmp3=", tmp3)
    tmp4= tmp3.replace(", ':'," , ":") #print("tmp4=", tmp4)
    tmp5= tmp4.replace(",'" , "'") #print("tmp5=", tmp5)
    tmp6= tmp5.replace("[" , "{") #open close for data dictionary input #print("tmp6=", tmp6)
    tmp7= tmp6.replace("]" , "}") #print("tmp7=", tmp7)
    tmp8= tmp7.replace("\'" , "\"") #print("tmp8=", tmp8) # worked with pyG5 up to here
    # end of pyG5 text mangling
    tmp9= tmp8.replace(": \"" , ": ") #  ' to " keys for pyefis #print("tmp9=", tmp9)
    tmp10= tmp9.replace("\"," , ",") #  ", to , keys for pyefis #print("tmp10=", tmp10)
    tmp11= tmp10.replace("\"}" , "}") # end quote and curly bracket -> bracket for pyefis #print("tmp11=", tmp11)
    tmp12= tmp11.replace("nan" , "0.0") # end quote and curly bracket -> bracket for pyefis #print("tmp11=", tmp11)
    #
    # send data to pyG5
    if(len(pyG5SimAddrPort) != 0):
        # convert text string from above into a data_dict{}
        data_dict.update(json.loads(tmp8))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as conn:
                try:       
                    serialized_data = pickle.dumps(data_dict)   
                    conn.sendto(serialized_data, (pyG5SimAddr, pyG5SimPort))  
                    print("--DRONEKIT-- SEND data_dict to PyG5 len(serialized_data), \
                        len(data_dict) ",len(serialized_data),len(data_dict))  
                except:
                    print("-EEEE-ERROR-: SendAttitudeDataToG5simEfisSimDronekit()  ERROR Sending pickle serialized data... ") 
        conn.close
    #
    # send data to pyEfis
    if(len(pyefisSimAddrPort) != 0):
        # convert text string from above into a data_dict{}
        data_dict.update(json.loads(tmp12))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as conn:
                try:       
                    serialized_data = pickle.dumps(data_dict)   
                    conn.sendto(serialized_data, (pyefisSimAddr, pyefisSimPort))   
                    print("--DRONEKIT-- SEND data_dict to pyEfis len(serialized_data), \
                    #      len(data_dict) " ,len(serialized_data),len(data_dict))       
                except:
                    print("-EEEE -ERROR-: SendAttitudeDataToG5simEfisSim  ERROR Sending pickle serialized data... ") 
        conn.close
    return

def SendAttitudeDataToG5simEfisSimMAVSDK(msg): # format and load as dict and then pickle and send to pyG5 or pyEfis
        global data_dict
        tmp = str(msg).split(" ") #print("tmp[1:]=", tmp[1:])
        text = re.sub('(\w+)\s?:\s?("?[^",]+"?,?)', "\"\g<1>\":\g<2>", str(tmp[1:])) #print("text ",text)
        tmp1= str(text).replace("[\'[" , "{") 
        tmp2= tmp1.replace("]\']" , "}")
        #print("tmp2:", tmp2)
        tmp3 = tmp2.replace(":\', \'",":")
        #print("tmp3:", tmp3)
        tmp4 = tmp3.replace(",\', \'",",") # ,', '
        #print("tmp4=", tmp4)
        tmp5 = tmp4.replace("\']" , "") 
        tmp6 = tmp5.replace("[\'" , "") 
        #print("tmp6:", tmp6)
        tmp8  = tmp6
        tmp11 = tmp6
        # send data to pyG5
        if(len(pyG5SimAddrPort) != 0):
            # convert text string from above into a data_dict{}
            data_dict.update(json.loads(tmp8))
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as conn:
                    try:       
                        serialized_data = pickle.dumps(data_dict)   
                        conn.sendto(serialized_data, (pyG5SimAddr, pyG5SimPort))  #(pyG5SimAddr, pyG5SimPort))  # pyG5SimAddr 
                        #print("--MAVSDK-- SEND data_dict to PyG5 len(serialized_data), \
                        #      len(data_dict) ",len(serialized_data),len(data_dict))    
                    except:
                        print("-EEEE-ERROR-: SendAttitudeDataToG5simEfisSimMAVSDK()  ERROR Sending pickle serialized data... ") 
                        #print("pyG5SimAddrPort-",pyG5SimAddrPort)
            conn.close
        #
        # send data to pyEfis
        if(len(pyefisSimAddrPort) != 0):
            # convert text string from above into a data_dict{}
            data_dict.update(json.loads(tmp11))
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as conn:
                    try:       
                        serialized_data = pickle.dumps(data_dict)   
                        conn.sendto(serialized_data, (pyefisSimAddr, pyefisSimPort))  #(pyG5SimAddr, pyG5SimPort))  # pyG5SimAddr  
                        #print("--MAVSDK-- SEND data_dict to pyEfis len(serialized_data), \
                        #      len(data_dict) " ,len(serialized_data),len(data_dict)) 
                    except:
                        print("-EEEE -ERROR-: SendAttitudeDataToG5simEfisSim  ERROR Sending pickle serialized data... ") 
                        #print("pyefisSimAddr-",pyefisSimAddr)
            conn.close
        return
################################################################################################
################################################################################################
################################################################################################
#useMAVSDK = False
if (useDronekit == True):
    # for dronekit
    # Import DroneKit-Python
    from dronekit import connect, Command, LocationGlobal # from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command   
    from pymavlink import mavutil

    '''# changes made to /usr/lib/python3.10/collections/__init__.py
    from _collections_abc import MutableMapping 
    import _collections_abc 

    #import _collections_abc # orig'''
    ################################################################################################
    # connect to MAVSDK libs version
    ################################################################################################
    #if (useMAVSDK == True):
        #if __name__ == "__main__":
            # Start the main function
            #syncio.run(run())
    # else connect to Dronekit below
    ################################################################################################
    # connect to MAVSDK libs version
    ################################################################################################
    # Connect to Dronekit
    ################################################################################################
    # open connection to autopilot sitl ardupilot px4
    print ("DRONEKIT: libs Connecting... to autopilot/sitl")
    vehicle = None
    # open using mavutil.mavlink_connection() open connection for mavlink msgs
    vehicle = mavutil.mavlink_connection(connection_string ,source_system=250, wait_ready=True,  ) 
    if(vehicle): print("Connection made to vehicle, info: (connection_string %s targ.system %u targ.component %u src.sys %u src.comp %u src.vehicle %s" %  (connection_string, vehicle.target_system, vehicle.target_component, vehicle.source_system, vehicle.source_component, vehicle))
    # Display basic vehicle state
    if(vehicle):
        print("Connected to vehicle, info: (connection_string %s targ.system %u targ.component %u src.sys %u src.comp %u src.vehicle %s" %  (connection_string, vehicle.target_system, vehicle.target_component, vehicle.source_system, vehicle.source_component, vehicle))
        #print (" Type: %s" % vehicle._vehicle_type)
        #print (" Armed: %s" % vehicle.armed)
        #print (" System status: %s" % vehicle.system_status.state)
        #print (" GPS: %s" % vehicle.gps_0)
        #print (" Alt: %s" % vehicle.location.global_relative_frame.alt)

    while(vehicle):
        # get mavlink message forever convert into data dict and pickle and send to pyG5 and pyEfis
        # if mavlink type 30 or 33 27 225 226 mavlink attitude or int_gps message send data to pyG5
        # send over tcp udp as 'pickle'd' data dictionary
        msg = vehicle.recv_match(blocking=True) 
        #msg = vehicle.recv_msg() 
        msgID = msg.get_msgId() 
        #print("--@@@@--Received mav_msgId:",msgID ," convert, sending to G5 and or Efis(): ", date_time," ", msg," ", end="")
        time.sleep(.005)
        if (msg != None):
            #if (msg != None): print("msg: ", msg)
            msgID = msg.get_msgId()  
            if (msg != "") and ( (msgID == 30) or (msgID == 33) or (msgID == 141) or (msgID == 27) or (msgID == 225) or (msgID == 226)):
                now = datetime.now()
                date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
                #print("-Received mav_msg, convert, send to G5 & Efis() ", date_time) #, msg)
                #print("\n--@@@@@@--Received from ",Px4simAddrPort," msgId:",msgID," sending to G5 and or Efis(): ", date_time) #," ", msg," ", end="")
                SendAttitudeDataToG5simEfisSimDronekit(msg)
                # MAVLINK_MSG_ID_ATTITUDE 30
                # MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33
                # MAVLINK_MSG_ID_RAW_IMU 27
                # MAVLINK_MSG_ID_ALTITUDE 141
                # MAVLINK_MSG_ID_RPM 226  
    
    # Disarm vehicle
    vehicle.armed = False
    time.sleep(1)

    # Close vehicle object before exiting script
    vehicle.close()
    time.sleep(1) 

    app.exec() # loop here until quit() or forever else run asyn version below instead if (useDronekit == False):
#the input connect string uses MAVSDK if //: in connect string
#

################################################################################################
################################################################################################
################################################################################################
# if (useDronekit == False):
###############################################################################################################################
###############################################################################################################################
# using MAVSDK libs packages to connect to PX4 and send data dict to pyG and pyEfis , also 
# if usb controller lie Great planes I-controller then can be used as manual input
# manual input commands use joystick axis
###############################################################################################################################
# https://docs.qgroundcontrol.com/Stable_V4.3/en/qgc-user-guide/setup_view/joystick.html
"""
how to use the manual controls with Joysticks plugin:

Note: Manual inputs are taken from a test set in this example to decrease
complexity. Manual inputs can be received from devices such as a joystick
using third-party python extensions.

Note: Taking off the drone is not necessary before enabling manual inputs.
It is acceptable to send positive throttle input to leave the ground.
Takeoff is used in this example to decrease complexity
"""
# if ("://" in Px4simAddrPort) string : # then set useMAVSDK = True
#if useMAVSDK: print("useMAVSDK = True")
#
import asyncio
import random
from mavsdk import System
import time
import pygame # joystick RC i-controller great planes

# timestamp date_time
from datetime import datetime, timedelta
### INIT global vars etc
pygame.display.init() #? required
pygame.joystick.init()

#Axis = []
Joysticks={}
PrevJoystickVals = {} # store prev vals
done = False 
roll=0.0; pitch=0.0; throttle=0.0; yaw=0.0
# globsl print vars
RxMAVmsg     = ""
RxMAVlen     = 0
RxMAVpkts    = 0
TxG5pkts     = 0
TxG5len      = 0
TxEfispkts   = 0
TxEfislen    = 0
RxJoyData    = ""
RxJoyUSBPkts = 0
TxJoyMAVPkts = 0
RxJoysticks  = {} # rx dict can be multiple Joysticks by guid, we dont care yet here
#
# print multiple lines
def PRINT():
    # timestamp date_time
    now = datetime.now()
    date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
    print(
        f'{date_time} MAVSDK RxMAVpkts={RxMAVpkts} RxMAVlen={RxMAVlen}',
        f'TxG5pkts={TxG5pkts} TxG5len={TxG5len}',
        f'TxEfispkts={TxEfispkts} TxEfislen={TxEfislen}',
        f'RxJoyUSBPkts={RxJoyUSBPkts} TxJoyMAVPkts={TxJoyMAVPkts} RxJoyData:{RxJoyData}',
        flush=False, end='\r'
    )
    #print()

#
async def run(): #run manual_controls()
    """Main function to connect to the drone and input manual controls"""
    
    # Connect to the Simulation
    drone = System()
    #
    print("MAVSDK Waiting for drone to connect...")
    await drone.connect(system_address="udp://:14540")
    #print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"drone().is_connected = true!")
            break
    # This waits till a mavlink based drone is connected
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    # Checking if Global Position Estimate is ok
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    info = await drone.info.get_version() # firmware version
    print("firmware version", info)
    #
    '''//Iterate through and detect systems connected
    for (auto system : mavsdk.systems()) {
        std::cout << "Found system with MAVLink system ID: " << static_cast<int>(system->get_system_id())
              << ", connected: " << (system->is_connected() ? "yes" : "no")
              << ", has autopilot: " << (system->has_autopilot() ? "yes" : "no") << '\n';
    }'''
    # https://docs.px4.io/main/en/advanced_config/parameter_reference.html
    '''
    # to test manual control arm takeoff wait for hold go to manual input
    # Arming the drone
    print("-- Arming")
    await drone.action.arm()

    # Takeoff the vehicle
    print("-- Taking off")
    await drone.action.takeoff()
    time.sleep(15) # wait to get to position hold
    '''
    #print("-- Starting manual control")
    #await drone.manual_control.set_manual_control_input(float(0), float(0), float(0.5), float(0)) #needs this ?
    
    #print("-- Starting altitude hold control")
    #await drone.manual_control.start_altitude_control()
    #
    # send mavlink messages from usb josticks
    asyncio.ensure_future(joystick_event(drone))
    asyncio.ensure_future(print_position(drone))
    asyncio.ensure_future(print_attitude_euler(drone))
    asyncio.ensure_future(print_heading(drone))
    while True:
        await asyncio.sleep(1)

async def joystick_event(drone):
    global roll, pitch, throttle, yaw, PrevJoystickVals, RxJoyData, TxJoyMAVPkts
    #print("--MAVSDK--async joystick_event()")
    while(True):
        # Get count of Joysticks.
        joystick_count = pygame.joystick.get_count()
        JoyAxes = joystick_count   #pygame.joystick.Joystick(0).get_numaxes()
        #print(f"Number of Joysticks: {joystick_count}")
        done = False
        dec = 3 # round to decimal
        while not done: # main joystick event loop
            #print("running manual control joystick loop ...") 
            await asyncio.sleep(0.1) # ten times a sec this code is executed
            # Event processing step.# Possible joystick events: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
            # JOYBUTTONUP, JOYHATMOTION, JOYDEVICEADDED, JOYDEVICEREMOVED
            # and for each joystick create a dict entry using guid as key and object addr and guid and name etc as values as a list
            # check if current vals different tahn prev vals            

            for i, j in PrevJoystickVals.items():
                if (True): 
                    now = datetime.now()
                    date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
                    #print(f"--MAVSDK-- SENDing 1/10 sec MAN CTL async JOYSTICK event: date:{date_time}, guid:{i}, roll:{Joysticks[i][5]}, pitch:{Joysticks[i][6]}, throttle:{Joysticks[i][7]},  yaw:{Joysticks[i][9]}" )
                    TxJoyMAVPkts += 1
                    RxJoyData = f" roll:{Joysticks[i][5]} pitch:{Joysticks[i][6]} throttle:{Joysticks[i][7]} yaw:{Joysticks[i][9]} sw#:{Joysticks[i][11]};{Joysticks[i][12]}.{Joysticks[i][13]}.{Joysticks[i][14]}.{Joysticks[i][15]}.{Joysticks[i][16]},guid:{i}"
                    await drone.manual_control.set_manual_control_input( float(Joysticks[i][5]), float(Joysticks[i][6]), float(Joysticks[i][7]), float(Joysticks[i][9]) )
                    #only if changged below
                    if  (round(float(PrevJoystickVals[i][5]), dec) != round(float(Joysticks[i][5]), dec)) or \
                    (round(float(PrevJoystickVals[i][6]), dec) != round(float(Joysticks[i][6]), dec)) or \
                    (round(float(PrevJoystickVals[i][7]), dec) != round(float(Joysticks[i][7]), dec)) or \
                    (round(float(PrevJoystickVals[i][8]), dec) != round(float(Joysticks[i][8]), dec)) or \
                    (round(float(PrevJoystickVals[i][9]), dec) != round(float(Joysticks[i][9]), dec)) or \
                    (round(float(PrevJoystickVals[i][10]), dec) != round(float(Joysticks[i][10]), dec) ) or \
                    ( PrevJoystickVals[i][12] != Joysticks[i][12] ) or \
                    ( PrevJoystickVals[i][13] != Joysticks[i][13] ) or \
                    ( PrevJoystickVals[i][14] != Joysticks[i][14] ) or \
                    ( PrevJoystickVals[i][15] != Joysticks[i][15] ) or \
                    ( PrevJoystickVals[i][16] != Joysticks[i][16] ) :
                        #now = datetime.now()
                        #date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
                        #print(f"--MAVSDK--async joystick_event() CHANGE: date:{date_time}, guid:{i}, roll:{Joysticks[i][5]}, pitch:{Joysticks[i][6]}, throttle:{Joysticks[i][7]}, yaw:{Joysticks[i][9]}, #sw:{Joysticks[i][11]}, sw1:{Joysticks[i][12]}, sw2:{Joysticks[i][13]}, sw3:{Joysticks[i][14]}, sw4:{Joysticks[i][15]}, sw5:{Joysticks[i][16]}" )
                        RxJoyData = f"--MAVSDK--async joystick_event() CHANGE: date:{date_time}, guid:{i}, roll:{Joysticks[i][5]}, pitch:{Joysticks[i][6]}, throttle:{Joysticks[i][7]}, yaw:{Joysticks[i][9]}, #sw:{Joysticks[i][11]}, sw1:{Joysticks[i][12]}, sw2:{Joysticks[i][13]}, sw3:{Joysticks[i][14]}, sw4:{Joysticks[i][15]}, sw5:{Joysticks[i][16]}"
            # check if current vals different tahn prev vals
            PrevJoystickVals.clear() # clears prev vals in prev vals dict
            for i, j in Joysticks.items():
                #print ("joystick:",str(i)," vals:",j)
                PrevJoystickVals[i] =  j # add current vals into old vals before rewrting new vals from joystick
            #
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True  # Flag that we are done so we exit this loop.

                if event.type == pygame.JOYBUTTONDOWN:
                    print("Joystick button pressed.")
                    #if event.button == 0:
                        #joystick = Joysticks[event.device_index] #instance_id]
                        #if joystick.rumble(0, 0.7, 500):
                            #print(f"Rumble effect played on joystick {event.instance_id}")

                if event.type == pygame.JOYBUTTONUP:
                    print("Joystick button released.")

                # Handle hotplugging (adding removing in real time usb Joysticks
                if event.type == pygame.JOYDEVICEADDED:
                    # This event will be generated when the program starts for every
                    # joystick, filling up the list without needing to create them manually.
                    joy = pygame.joystick.Joystick(event.device_index)
                    Joysticks.update({joy.get_guid():[joy]}) # add object addr val to key=guid dict Joysticks
                    print ("ADDING NEW joystick key:",joy.get_guid(),"  val:",[joy])
                    for i, j in Joysticks.items():
                        print ("ADDED NEW joystick dict:",str(i)," vals:",j)
                    print(f"ADDED Joystick {joy.get_instance_id()} is now connected {Joysticks.keys()} ; {Joysticks.values()} ; {Joysticks.items()}" )
                    joystick_count = pygame.joystick.get_count()

                if event.type == pygame.JOYDEVICEREMOVED:
                    for k, v in Joysticks.items(): #for joystick in Joysticks:  #.values():   #for joystick in Joysticks:
                        jid = v[1] #.get_instance_id()
                        guid = k # keys are guid from pygame joy.get_guid()
                        print ("DISCONNECTING guid:",guid,"  jid:",jid)
                        if (event.instance_id == jid): 
                            print ("DEL Joysticks[guid] dict guid, inst id jid:",guid, jid)
                            print(f"DISCONNECT Joystick {event.instance_id} is now disconnected")
                            del Joysticks[k] #event.instance_id] # joy.get_guid()
                            PrevJoystickVals = {} 
                            break # short circuit
            #
            for joystick in Joysticks.values():
                jid = joystick[0].get_instance_id()
                guid = joystick[0].get_guid() # from PYTHON OBJECT addr get guid from above when added new joystick event
                Joysticks.update( { guid : [ joystick[0] ] } ) # restart remove vals for key and replace with guid
                Joysticks[guid].append(jid)
                # Get the name from the OS for the controller/joystick.
                name = joystick[0].get_name() #print(f"Joystick name: {name}")
                Joysticks[guid].append(name) #print("add name Joysticks[jid]:",Joysticks[jid])

                #guid = joystick[0].get_guid()
                #print(f"GUID: {guid}")
                #Joysticks[guid].append(guid)
                #print("add GUID Joysticks[jid]:",Joysticks[jid])

                power_level = joystick[0].get_power_level()  #print(f"Joystick's power level: {power_level}")
                Joysticks[guid].append(power_level)  #print("add power_level Joysticks[jid]:",Joysticks[jid])

                # Usually axis run in pairs, up/down for one, and left/right for
                # the other. Triggers count as axes.
                axes = joystick[0].get_numaxes()
                #print(f"Number of axes: {axes}")
                Joysticks[guid].append(axes)
                #print("add axes Joysticks[jid]:",Joysticks[jid])

                for i in range(axes):
                    axis = joystick[0].get_axis(i)
                    #print(f"Axis {i} value: {axis:>6.3f}")
                    Joysticks[guid].append(round(axis,5))
                    #print(f"add axis {i} {Joysticks[jid]}")

                buttons = joystick[0].get_numbuttons()
                #print(f"Number of buttons: {buttons}")
                Joysticks[guid].append(buttons)
                #print(f"add buttons {buttons} {Joysticks[jid]}")

                for i in range(buttons):
                    button = joystick[0].get_button(i)
                    #print(f"Button {i:>2} value: {button}")
                    Joysticks[guid].append(button)
                    #print(f"add button {i} {Joysticks[jid]}")

                hats = joystick[0].get_numhats()
                #print(f"Number of hats: {hats}")
                Joysticks[guid].append(hats)
                #print(f"add hats {hats} {Joysticks[jid]}")

                # Hat position. All or nothing for direction, not a float like
                # get_axis(). Position is a tuple of int values (x, y).
                for i in range(hats):
                    hat = joystick[0].get_hat(i)
                    #print(f"hat: {hat}")
                    Joysticks[guid].append(hat) # comes back as a list, not an int value
                    #print(f"Hat {i} value: {str(hat)}")
                    #print(f"add hat {i} {Joysticks[jid]}")
        ## END JOYSTICK EVENT ####

async def print_position(drone):
    global RxMAVlen, RxMAVpkts, RxMAVmsg, TxEfispkts, TxEfislen, TxG5pkts, TxG5len
    async for position in drone.telemetry.position():
        #print("print_position():", position)
        #now = datetime.now()
        #date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
        #print("\n--@@@@@@--Received from ",Px4simAddrPort," MAVSDK position msg converting") #, sending to G5 and or Efis(): ", date_time," ", position)
        #print("--MAVSDK-- RECVd position msg",  position )
        RxMAVlen    = len(str(position))
        RxMAVpkts  +=1
        RxMAVmsg    = position
        TxEfispkts +=1
        TxEfislen   = len(str(position))
        TxG5pkts   +=1
        TxG5len     = len(str(position))
        PRINT()
        SendAttitudeDataToG5simEfisSimMAVSDK(position)

async def print_heading(drone):
    global RxMAVlen, RxMAVpkts, RxMAVmsg, TxEfispkts, TxEfislen, TxG5pkts, TxG5len
    async for heading in drone.telemetry.heading()  :
        #print(f"print_heading(): {heading}") 
        #now = datetime.now()
        #date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
        #print("\n--@@@@@@-Received from ",Px4simAddrPort," MAVSDK heading converting") #, sending to G5 and or Efis(): ", date_time," ", heading)
        #print("--MAVSDK-- RECVd heading msg",  heading )
        RxMAVlen = len(str(heading))
        RxMAVpkts +=1
        RxMAVmsg = heading
        TxEfispkts +=1
        TxEfislen   = len(str(heading))
        TxG5pkts   +=1
        TxG5len     = len(str(heading))
        PRINT()
        SendAttitudeDataToG5simEfisSimMAVSDK(heading)

async def print_attitude_euler(drone):
    global RxMAVlen, RxMAVpkts, RxMAVmsg, TxEfispkts, TxEfislen, TxG5pkts, TxG5len
    async for attitude_euler in drone.telemetry.attitude_euler()  :
        #print("print_attitude_euler():", attitude_euler )
        #now = datetime.now()
        #date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
        #print("\n--@@@@@@--Received from ",Px4simAddrPort," MAVSDK attitude_euler converting") #", sending to G5 and or Efis(): ", date_time," ", attitude_euler)
        #print("--MAVSDK-- RECVd attitude_euler msg",  attitude_euler )
        RxMAVlen = len(str(attitude_euler))
        RxMAVpkts +=1
        RxMAVmsg = attitude_euler
        TxEfispkts +=1
        TxEfislen   = len(str(attitude_euler))
        TxG5pkts   +=1
        TxG5len     = len(str(attitude_euler))
        PRINT()
        SendAttitudeDataToG5simEfisSimMAVSDK(attitude_euler)

async def print_battery(drone):
    async for battery in drone.telemetry.battery():
        print(f"--MAVSDK-- RECVd Battery{battery} %:{battery.remaining_percent}")


async def print_gps_info(drone):
    async for gps_info in drone.telemetry.gps_info():
        print(f"--MAVSDK-- RECVd GPS_info msg:{gps_info}")


async def print_in_air(drone):
    async for in_air in drone.telemetry.in_air():
        print(f"In air: {in_air}")

###############################################################################################################################



if __name__ == "__main__":
    # Start the main function
    asyncio.run(run())
