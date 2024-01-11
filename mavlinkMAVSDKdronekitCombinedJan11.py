#!/usr/bin/env python3
from platform import python_version
print("using Python ver:",python_version())
#
# for mavsdk
import asyncio
#from mavsdk import start_mavlink
from mavsdk import System
#from mavsdk import connect

# for dronekit
# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal # from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command   
from pymavlink import mavutil

'''# changes made to /usr/lib/python3.10/collections/__init__.py
from _collections_abc import MutableMapping 
import _collections_abc 

#import _collections_abc # orig'''

from math import *

from datetime import datetime, timedelta
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

import time, sys, argparse
import socket
import pickle
import subprocess 
import json
import re
# keyboard events
import KeyPressModule as kp

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
# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py
# https://firmware.ardupilot.org/Plane/latest/Pixhawk1-1M/  ARDUPLANE #4.5.0-FIRMWARE_VERSION_TYPE_DEV
# https://github.com/ArduPilot/pymavlink/blob/master/mavextra.py
# USEAGE
# $ python telemetry.py -m "serial:///dev/ttyUSB0:57600" -g 127.0.0.1:65432 -e 127.0.0.1:2100
# $ python3 FIX-Gateway/mavlink2PX4G5.py -m "/dev/ttyUSB0,57600" -g 127.0.0.1:65432 -e 127.0.0.1:2100
# $ python telemetry.py -m "udp://:14540" -g 127.0.0.1:65432 -e 127.0.0.1:2100
################################################################################################
#$ python telemetry.py -m "udp://:14540" -g 127.0.0.1:65432 -e 127.0.0.1:2100
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
#$ python telemetry.py -m ":14445" -g 127.0.0.1:65432 -e 127.0.0.1:2100
#$ python telemetry.py -m "udp://:14445" -g 127.0.0.1:65432 -e 127.0.0.1:2100
#python3 ./mavlink2pyg5.py -m /dev/ttyUSB0,57600 -g 127.0.0.1:65432 -e 127.0.0.1:2100 pixhawk usb  
#python3 ./mavlink2pyg5.py -m /dev/ttyACM0,57600 -g 127.0.0.1:65432 -e 127.0.0.1:2100  sik radio  
#python3 ./mavlink2pyg5.py -m 172.17.0.1:14550 -g 127.0.0.1:65432 -e 127.0.0.1:2100 sitl  
#python3 ./mavlink2PX4G5.py -m 127.0.0.1:14445 -g 127.0.0.1:65432 -e 127.0.0.1:2100 qgcs forwarded  
#to starrt sitl px4 gazebo sim locally via docker app:  
#$ sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2  
################################################################################################

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
                    conn.sendto(serialized_data, (pyG5SimAddr, pyG5SimPort))  #(pyG5SimAddr, pyG5SimPort))  # pyG5SimAddr
                    #msg.get_payload() ) #create byte wise stream from string text   
                    print("--%%%%-- Sending pickled data_dict to PyG5 len(serialized_data), len(data_dict) ",len(serialized_data),len(data_dict))  #," ,pyG5SimAddrPort-",pyG5SimAddrPort,"=pyG5SimAddr, ",pyG5SimAddr,"=pyG5SimPort:",pyG5SimPort, tmp8)
                    #print("-%%%%- Sending pickled data_dict ... len(serialized_data), data_dict len= ",len(serialized_data), data_dict)                
                except:
                    print("-EEEE-ERROR-: SendAttitudeDataToG5simEfisSimDronekit()  ERROR Sending pickle serialized data... ") 
                   #print("pyG5SimAddrPort-",pyG5SimAddrPort)
        conn.close
    #
    # send data to pyEfis
    if(len(pyefisSimAddrPort) != 0):
        # convert text string from above into a data_dict{}
        data_dict.update(json.loads(tmp12))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as conn:
                try:       
                    serialized_data = pickle.dumps(data_dict)   
                    conn.sendto(serialized_data, (pyefisSimAddr, pyefisSimPort))  #(pyG5SimAddr, pyG5SimPort))  # pyG5SimAddr
                    #msg.get_payload() ) #create byte wise stream from string text   
                    print("--$$$$-- Sending pickled data_dict to pyEfis len(serialized_data), len(data_dict) " ,len(serialized_data),len(data_dict)) #, pyefisSimAddr, "=pyefisSimAddr, ",  pyefisSimPort,"=pyefisSimPort", tmp11)
                    #print("-$$$$- Sending pickled data_dict ... len(serialized_data), data_dict len= ",len(serialized_data), data_dict)                
                except:
                    print("-EEEE -ERROR-: SendAttitudeDataToG5simEfisSim  ERROR Sending pickle serialized data... ") 
                    #print("pyefisSimAddr-",pyefisSimAddr)
        conn.close
    return

def SendAttitudeDataToG5simEfisSimMAVSDK(msg): # format and load as dict and then pickle and send to pyG5 or pyEfis
        global data_dict
        #now = datetime.now()
        #date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
        #print("--MAVSDK--Rxd:", len(str(msg)), end="\r") #print("--MAVSDK--Recvd:", end="\r") , str(msg).split(":")[0] , len(str(msg))
        #print("--MAVSDK--Recvd msg",  msg )
        tmp = str(msg).split(" ") #print("tmp[1:]=", tmp[1:])
        text = re.sub('(\w+)\s?:\s?("?[^",]+"?,?)', "\"\g<1>\":\g<2>", str(tmp[1:])) #print("text ",text)
        #print("testmsg:",text)
        '''
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
        '''
        #mesgStr = str(msg)
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
                        #msg.get_payload() ) #create byte wise stream from string text   
                        print("--%%%%-- Sending pickled data_dict to PyG5 len(serialized_data), len(data_dict) ",len(serialized_data),len(data_dict)) #," ,pyG5SimAddrPort-",pyG5SimAddrPort,"=pyG5SimAddr, ",pyG5SimAddr,"=pyG5SimPort:",pyG5SimPort)
                        #print("-%%%%-len(pyG5)",len(serialized_data), end="")        
                        #print("-%%%%-len(pyG5)",len(serialized_data), end=" ")         
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
                        #msg.get_payload() ) #create byte wise stream from string text   
                        print("--$$$$-- Sending pickled data_dict to pyEfis len(serialized_data), len(data_dict) " ,len(serialized_data),len(data_dict)) #, pyefisSimAddr, "=pyefisSimAddr, ",  pyefisSimPort,"=pyefisSimPort")
                        #print("-$$$$-len(pyEfis)",len(serialized_data), end=" ")                
                    except:
                        print("-EEEE -ERROR-: SendAttitudeDataToG5simEfisSim  ERROR Sending pickle serialized data... ") 
                        #print("pyefisSimAddr-",pyefisSimAddr)
            conn.close
        #print(date_time,"    ", end="\r")
        return

#useMAVSDK = False
if (useDronekit == True):
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

    app.exec()
# loop here until quit or forever else run asyn version below
###############################################################################################################################
###############################################################################################################################
###############################################################################################################################
roll, pitch, throttle, yaw # global 
async def run():
    kp.init() # init keyboard events
    # connect via mavsdk()
    # Init the drone
    drone = System()
    print("MAVSDK: libs Connecting... to autopilot/sitl")
    # px4 sitl sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2
    await drone.connect(Px4simAddrPort) #connection_string) # "udp://:14540") # yup 
    # nope await drone.connect(system_address="udp:172.17.0.1:14540") #"udp://:14540")
    print("connected !")
    #await drone.action.arm()
    #print("arm...")
    #await drone.action.takeoff()
    #print("takeoff...")
    info = await drone.info.get_version() # firmware version
    print("firmware version", info)
    #attitudeEuler = await drone.telemetry.attitude_euler()   
    #print("attitudeEuler:",attitudeEuler)
    #
    #async for position in drone.telemetry.position():
        #print("position:",position)
        #heading == async drone.telemetry.heading()  
        #print("heading:",heading)
        #attitudeEuler = drone.telemetry.attitude_euler()   
        #print("attitudeEuler:",attitudeEuler)

        # set the manual control input after arming and takeof


    # start manual control
    print("-- Starting manual control - start_altitude_control()")
    #await drone.manual_control.start_position_control()
    await drone.manual_control.start_altitude_control()

        #set the manual control input after arming and takeoff
    #await drone.manual_control.set_manual_control_input( float(0), float(0), float(0.5), float(0) )

    # Start the tasks
    #asyncio.ensure_future(print_battery(drone))
    #asyncio.ensure_future(print_gps_info(drone))
    #asyncio.ensure_future(print_in_air(drone))
    # mavlink messages
    asyncio.ensure_future(getKeyboardInput(drone))
    asyncio.ensure_future(joystick_event(drone))
    asyncio.ensure_future(print_position(drone))
    asyncio.ensure_future(print_attitude_euler(drone))
    asyncio.ensure_future(print_heading(drone))
    while True:
        await asyncio.sleep(1)

    
    async def getKeyboardInput(my_drone):
        '''#at startup import KeyPressModule as kp
        # Test set of manual inputs. Format: [roll, pitch, throttle, yaw]
        kp.init()'''
        global roll, pitch, throttle, yaw
        while True:
            roll, pitch, throttle, yaw = 0, 0, 0.5, 0
            value = 1
            if kp.getKey("LEFT"):
                pitch = -value
            elif kp.getKey("RIGHT"):
                pitch = value
            if kp.getKey("UP"):
                roll = value
            elif kp.getKey("DOWN"):
               roll = -value
            if kp.getKey("w"):
                throttle = value
            elif kp.getKey("s"):
                throttle = 0
            if kp.getKey("a"):
               yaw = -value
            elif kp.getKey("d"):
                yaw = value
            elif kp.getKey("i"):
                asyncio.ensure_future(print_flight_mode(my_drone))
            elif kp.getKey("q") and my_drone.telemetry.landed_state():
                await my_drone.action.arm()
            elif kp.getKey("l") and my_drone.telemetry.in_air():
                await my_drone.action.land()
        print(roll, pitch, throttle, yaw)
        await asyncio.sleep(0.1)

async def joystick_event(drone):
    global roll, pitch, throttle, yaw
    print("--MAVSDK--async joystick_event()")
    #async for position in drone.telemetry.position():
    #print("print_position():", position)
    #now = datetime.now()
    #date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
    #print("\n--@@@@@@--Received from ",Px4simAddrPort," MAVSDK position msg converting") #, sending to G5 and or Efis(): ", date_time," ", position)
    #SendAttitudeDataToG5simEfisSimMAVSDK(position)

async def print_position(drone):
    async for position in drone.telemetry.position():
        #print("print_position():", position)
        #now = datetime.now()
        #date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
        #print("\n--@@@@@@--Received from ",Px4simAddrPort," MAVSDK position msg converting") #, sending to G5 and or Efis(): ", date_time," ", position)
        print("--MAVSDK--Recvd position msg",  position )
        SendAttitudeDataToG5simEfisSimMAVSDK(position)

async def print_heading(drone):
    async for heading in drone.telemetry.heading()  :
        #print(f"print_heading(): {heading}") 
        #now = datetime.now()
        #date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
        #print("\n--@@@@@@-Received from ",Px4simAddrPort," MAVSDK heading converting") #, sending to G5 and or Efis(): ", date_time," ", heading)
        print("--MAVSDK--Recvd heading msg",  heading )
        SendAttitudeDataToG5simEfisSimMAVSDK(heading)

async def print_attitude_euler(drone):
    async for attitude_euler in drone.telemetry.attitude_euler()  :
        #print("print_attitude_euler():", attitude_euler )
        #now = datetime.now()
        #date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
        #print("\n--@@@@@@--Received from ",Px4simAddrPort," MAVSDK attitude_euler converting") #", sending to G5 and or Efis(): ", date_time," ", attitude_euler)
        print("--MAVSDK--Recvd attitude_euler msg",  attitude_euler )
        SendAttitudeDataToG5simEfisSimMAVSDK(attitude_euler)

async def print_battery(drone):
    async for battery in drone.telemetry.battery():
        print(f"--MAVSDK--Recvd Battery{battery} %:{battery.remaining_percent}")


async def print_gps_info(drone):
    async for gps_info in drone.telemetry.gps_info():
        print(f"--MAVSDK--Recvd GPS_info: {gps_info}")


async def print_in_air(drone):
    async for in_air in drone.telemetry.in_air():
        print(f"In air: {in_air}")

###############################################################################################################################



if __name__ == "__main__":
    # Start the main function
    asyncio.run(run())
