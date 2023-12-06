#!/usr/bin/env python
# if python3.10 then changes need in 
# home/jf/.local/lib/python3.10/site-packages/dronekit/__init__.py
# Import DroneKit-Python
from platform import python_version
print("using Python ver:",python_version())

from dronekit import connect, Command, LocationGlobal 
# from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command   
from pymavlink import mavutil
import time, sys, argparse, math 
import time
from datetime import datetime, timedelta
import socket
import pickle
import subprocess 
import json
import re
from math import *

# usage: $ python3 ./mavlink2PX4G5.py -m 172.17.0.1:14550 -g 127.0.0.1:65432 -e 127.0.0.1:2100

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

# timestamp date_time
now = datetime.now()
date_time = now.strftime("%m/%d/%Y, %H:%M:%S")

MAV_MODE_AUTO   = 4
data_dict = {}
# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py
# https://firmware.ardupilot.org/Plane/latest/Pixhawk1-1M/  ARDUPLANE #4.5.0-FIRMWARE_VERSION_TYPE_DEV
# https://github.com/ArduPilot/pymavlink/blob/master/mavextra.py

################################################################################################
# Parse connection argument
parser = argparse.ArgumentParser()
#parser.add_argument("-c", "--connect", help="connection string")
parser.add_argument("-m", "--px4", help="recv MAV from mavlink px4 ardupilot dronekit connection string /dev/ttyUSB0,57600, /dev/ttyACM0,57600 or sitl 172.17.0.1:14550, qgcs 127.0.0.1:14445")
parser.add_argument("-g", "--pyg5", help="pyg5sim sendto address:port 127.0.0.1:65432 same as xplane for now")
parser.add_argument("-e", "--pyefis", help="pyefis sendto address:port 127.0.0.1:2100")
args = parser.parse_args() # command line parameters
# PX4 input mavlink messages
if args.px4: # input from sitl qgcs or usb serial port to autopilot
    connection_string = args.px4
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

print("cli autopilot ardupilot dronekit connection string-Px4simAddrPort",Px4simAddrPort,"  pyG5SimAddrPort-",pyG5SimAddrPort,"  pyefisSimAddrPort-",pyefisSimAddrPort)
#python3 ./mavlink2pyg5.py -m /dev/ttyUSB0,57600 -g 127.0.0.1:65432 -e 127.0.0.1:2100 pixhawk usb  
#python3 ./mavlink2pyg5.py -m /dev/ttyACM0,57600 -g 127.0.0.1:65432 -e 127.0.0.1:2100  sik radio  
#python3 ./mavlink2pyg5.py -m 172.17.0.1:14550 -g 127.0.0.1:65432 -e 127.0.0.1:2100 sitl  
#python3 ./mavlink2PX4G5.py -m 127.0.0.1:14445 -g 127.0.0.1:65432 -e 127.0.0.1:2100 qgcs forwarded  
#to starrt sitl px4 gazebo sim locally via docker app:  
#$ sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2  
################################################################################################

def SendAttitudeDataToG5simEfisSim(msg): # format and load as dict and then pickle and send to pyG5 or pyEfis
    global data_dict
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
                    print("\n--%%%%-- Sending pickled data_dict to PyG5 len(serialized_data), data_dict len= ",len(serialized_data)," ,pyG5SimAddrPort-",pyG5SimAddrPort,"=pyG5SimAddr, ",pyG5SimAddr,"=pyG5SimPort:",pyG5SimPort)
                    #print("-%%%%- Sending pickled data_dict ... len(serialized_data), data_dict len= ",len(serialized_data), data_dict)                
                except:
                    print("-EEEE-ERROR-: SendAttitudeDataToG5simEfisSim()  ERROR Sending pickle serialized data... ") 
                    print("pyG5SimAddrPort-",pyG5SimAddrPort)
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
                    print("\n--$$$$-- Sending pickled data_dict to pyEfis len(serialized_data), data_dict len= " ,len(serialized_data), pyefisSimAddr, "=pyefisSimAddr, ",  pyefisSimPort,"=pyefisSimPort")
                    #print("-$$$$- Sending pickled data_dict ... len(serialized_data), data_dict len= ",len(serialized_data), data_dict)                
                except:
                    print("-EEEE -ERROR-: SendAttitudeDataToG5simEfisSim  ERROR Sending pickle serialized data... ") 
                    print("pyefisSimAddr-",pyefisSimAddr)
        conn.close
    return


################################################################################################
# Init
################################################################################################
# open connection to autopilot sitl ardupilot px4
print ("Connecting... to autopilot/sitl")
vehicle = None
# open using mavutil.mavlink_connection() open connection for mavlink msgs
vehicle = mavutil.mavlink_connection(connection_string ,source_system=250, wait_ready=True,  ) 
print("Connection made to vehicle, info: (connection_string %s targ.system %u targ.component %u src.sys %u src.comp %u src.vehicle %s" %  (connection_string, vehicle.target_system, vehicle.target_component, vehicle.source_system, vehicle.source_component, vehicle))
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
    msg = vehicle.recv_msg() 
    time.sleep(.005)
    if (msg != None):
        #if (msg != None): print("msg: ", msg)
        msgID = msg.get_msgId()  
        if (msg != "") and ( (msgID == 30) or (msgID == 33) or (msgID == 141) or (msgID == 27) or (msgID == 225) or (msgID == 226)):
            now = datetime.now()
            date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
            #print("-Received mav_msg, convert, send to G5 & Efis() ", date_time) #, msg)
            print("--@@@@--Received mav_msgId:",msgID," convert, sending to G5 and or Efis(): ", date_time," ", msg," ", end="")
            SendAttitudeDataToG5simEfisSim(msg)
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
############################### END #########################
# notes:
# https://dronekit-python.readthedocs.io/en/latest/guide/connecting_vehicle.html
# sudo tcpdump -i lo -n udp port 14445 port on jmavsim headless docker 127.0.0.1:14445 default forwarding by qgcs
# $ sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2
# sudo tcpdump -i lo -n udp port 14550 data from sitl at 172.17.0.1
# https://docs.qgroundcontrol.com/master/en/releases/daily_builds.html
# https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl.AppImage
# firmware ardupilot https://firmware.ardupilot.org/Plane/latest/Pixracer/
# https://ardupilot.org/dev/docs/pre-built-binaries.html
# https://firmware.ardupilot.org/Plane/stable/Pixracer-bdshot/arduplane.apj
# https://firmware.ardupilot.org/Plane/stable/
# subprocess.Popen(["sudo","docker run --rm -it jonasvautherin/px4-gazebo-headless"]) 
# $ sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2
# $ FIXGW ./fixgw.py -v -d -config-file "fixgw/configs/default.yaml"
# Connect to the Vehicle
# ~/fixgw$ python3 ./mavlink2pyg5.py -c 172.17.0.1:14550
# $ sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2
# jf@jfhome:~/fixgw$ sudo tcpdump -i lo -n udp port 14550 # from docker gazebo
# $ sudo tcpdump -i lo -n udp port 14445
# $ sudo tcpdump -i docker0 port 14550





