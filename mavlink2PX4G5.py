#!/usr/bin/env python
# Import DroneKit-Python
from platform import python_version
print(python_version())

try: # for dronekit after python 3.8
    # 👇️ using Python 3.10+
    from collections.abc import MutableMapping
except ImportError:
    # 👇️ using Python 3.10-
    from collections     import MutableMapping

# 👇️ <class 'collections.abc.MutableMapping'>
print(MutableMapping)
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

# jf@jfhome:~/fixgw$ python3 ./mavlink2pyg5.py -m 172.17.0.1:14550 -g 127.0.0.1:65432 -e 127.0.0.1:2100

# timestamp date_time
now = datetime.now()
date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
################################################################################################
# Settings
################################################################################################
#   vehSERPort = "/dev/ttyACM0"
#    vehSERBaud = "115200"
#python3 fullmission.py --c /dev/ttyACM0,57600 worked

vehIPaddr = "localhost:14445" #"/dev/ttyACM0,57600" # "127.0.0.1:14445" #"127.0.0.1" #"172.17.0.1" # dronekit connect port "172.17.0.1:14550" if qgcs "127.0.0.1:14445 # docker created addr "172.17.0.2:18570" jmavsim docker
#vehIPport =  "" #"14445"   #"14445" #"115200"  # :"14445"  #"14550"     # "14445"    # use "172.17.0.1:14445" if docker ## use "localhost:14450" qgcs running with forwarding 
Px4simAddrPort=""
pyG5SimAddrPort=""
pyefisSimAddrPort=""
'''
connection_string = vehIPaddr   # + ":" + vehIPport # '172.17.0.1:14550'  #'127.0.0.1:14445' connection_string = "/dev/ttyUSB0"
Px4simAddrPort = "172.17.0.1:14550" #"127.0.0.1:2100"
#
pyG5SimAddrPort = "127.0.0.1:65432"  # reuse xplane port for now
pyG5str = pyG5SimAddrPort.split(":")  #"127.0.0.1"  # The server's hostname or IP address
pyG5SimAddr = pyG5str[0] # 127.0.0.1
pyG5SimPort = pyG5str[1] # 2100 #65432  # The port used by the server
#
pyefisSimAddrPort = "127.0.0.1:2100"
pyefisSimStr = pyefisSimAddrPort.split(":")  #"127.0.0.1"  # The server's hostname or IP address
pyefisSimAddr = pyefisSimStr[0] # 127.0.0.1
pyefisSimPort = pyefisSimStr[1] # 2100 #65432  # The port used by the server
# '''
MAV_MODE_AUTO   = 4
data_dict = {}
# https://github.com/PX4/Firmware/blob/master/Tools/mavlink_px4.py
#https://firmware.ardupilot.org/Plane/latest/Pixhawk1-1M/  ARDUPLANE #4.5.0-FIRMWARE_VERSION_TYPE_DEV

# https://github.com/ArduPilot/pymavlink/blob/master/mavextra.py

################################################################################################
# Parse connection argument
parser = argparse.ArgumentParser()
#parser.add_argument("-c", "--connect", help="connection string")
parser.add_argument("-m", "--px4", help="mavlink px4 dronekit connection string or address:port 172.17.0.1:14550")
parser.add_argument("-g", "--pyg5", help="pyg5sim address:port 127.0.0.1:65432")
parser.add_argument("-e", "--pyefis", help="pyefis address:port 127.0.0.1:2100")
args = parser.parse_args()
pyG5SimAddr = ""
pyG5SimPort = 0
pyefisSimAddr = ""
pyefisSimPort = ""
if args.px4:
    connection_string = args.px4
if args.pyg5:
    pyG5SimAddrPort = args.pyg5 #pyG5SimAddrPort = "127.0.0.1:65432"  # reuse xplane port for now
    pyG5str = pyG5SimAddrPort.split(":")  #"127.0.0.1"  # The server's hostname or IP address
    pyG5SimAddr = pyG5str[0] # 127.0.0.1
    pyG5SimPort = int(pyG5str[1]) # 2100 #65432  # The port used by the server
    print("pyG5SimAddr -",pyG5SimAddr, "  pyG5SimPort-",pyG5SimPort)
if args.pyefis:
    pyefisSimAddrPort = args.pyefis
    pyefisSimStr = pyefisSimAddrPort.split(":")  #"127.0.0.1"  # The server's hostname or IP address
    pyefisSimAddr = pyefisSimStr[0] # 127.0.0.1
    pyefisSimPort = int(pyefisSimStr[1]) # 2100 #65432  # The port used by the server
    print("pyefisSimAddr-",pyefisSimAddr,"  pyefisSimPort-",pyefisSimPort)

print("autopilot ardupilot dronekit connection string-Px4simAddrPort",Px4simAddrPort,"  pyG5SimAddrPort-",pyG5SimAddrPort,"  pyefisSimAddrPort-",pyefisSimAddrPort)


def mag_heading_motors():
    global data_dict
    #print('calculate heading from raw magnetometer')
    heading = declination = 0
    #ofs = get_motor_offsets(SERVO_OUTPUT_RAW, ofs, motor_ofs)

    #if declination is None:
    #    from . import mavutil
    #    declination = degrees(mavutil.mavfile_global.param('COMPASS_DEC', 0))
    try:
        mag_x =  float(data_dict["xmag"].replace("'","")) 
    except:
        print('data_dict["xmag"] not exist')
        return heading 
    try:
        mag_y =  float(data_dict["ymag"].replace("'","")) 
    except:
        print('data_dict["ymag"] not exist')
        return heading
    try:
        mag_z =  float(data_dict["zmag"].replace("'","")) 
    except:
        print('data_dict["zmag"] not exist') 
        return heading 
    try:
        #Direction (y>0) = 90 - [arcTAN(x/y)]*180/π
        #Direction (y<0) = 270 - [arcTAN(x/y)]*180/π
        #Direction (y=0, x<0) = 180.0
        #Direction (y=0, x>0) = 0.0
        #
        #if (mag_y>0): heading = 90 - degrees(atan(mag_x/mag_y))*180/3.14159265359
        #if (mag_y<0): heading = 270 - degrees(atan(mag_x/mag_y))*180/3.14159265359
        #if (mag_y == 0)and(mag_x<0): heading=180
        #if (mag_y == 0)and(mag_x>0): heading=0
        heading = round((atan2(mag_y, mag_x) * 180 / 3.14159265359), 2)
        # put this calc heading into data dictionary
        data_dict["hdg"] = str(heading)  #str(heading) #18000 #int(heading * 100 )
        print("mag_x,y,z , ", mag_x, mag_y, mag_z, "  ,heading", data_dict["hdg"])
    except:
        #data_dict["hdg"] = str(heading)  #str(heading) #18000 #int(heading * 100 )
        #print("calculated heading= , ", heading, "  data_dict[hdg]=", data_dict["hdg"])
        print("roll or pitch error  data not exist yet avail at startup only heading= ", heading)
        return heading
    
def mag_field():
    '''calculate magnetic field strength from raw magnetometer'''
    '''xmag': '-590', 'ymag': '-603', 'zmag': '392'''
    mag_x =  float(data_dict["xmag"].replace("'","")) 
    mag_y =  float(data_dict["ymag"].replace("'","")) 
    mag_z =  float(data_dict["zmag"].replace("'","")) 
    return sqrt(mag_x**2 + mag_y**2 + mag_z**2)

def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)



def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)

def SendAttitudeDataToG5simEfisSim(msg):
    global data_dict
    #print("sending to pyG5 dat in msg \n", msg)
    #text = 'storeName: "testName", '

    tmp = str(msg).split(" ")
    #print("tmp[1:]=", tmp[1:])
    text = re.sub('(\w+)\s?:\s?("?[^",]+"?,?)', "\"\g<1>\":\g<2>", str(tmp[1:]))
    #print("text ",text)
    tmp2= str(tmp[1:]).replace("'{","'")
    #print("tmp2=", tmp2)
    tmp3= tmp2.replace("}'","'")
    #print("tmp3=", tmp3)
    tmp4= tmp3.replace(", ':'," , ":")
    #print("tmp4=", tmp4)
    tmp5= tmp4.replace(",'" , "'")
    #print("tmp5=", tmp5)
    tmp6= tmp5.replace("[" , "{") #open close for data dictionary input
    #print("tmp6=", tmp6)
    tmp7= tmp6.replace("]" , "}")
    #print("tmp7=", tmp7)
    tmp8= tmp7.replace("\'" , "\"")
    #print("tmp8=", tmp8) # worked with pyG5 up to here
    if(len(pyG5SimAddrPort) != 0):
        data_dict.update(json.loads(tmp8))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as conn:
                try:       
                    serialized_data = pickle.dumps(data_dict)   
                    conn.sendto(serialized_data, (pyG5SimAddr, pyG5SimPort))  #(pyG5SimAddr, pyG5SimPort))  # pyG5SimAddr
                    #msg.get_payload() ) #create byte wise stream from string text   
                    print("\n--%%%%- Sending pickled data_dict to PyG5 len(serialized_data), data_dict len= ",len(serialized_data)," ,pyG5SimAddrPort-",pyG5SimAddrPort,"=pyG5SimAddr, ",pyG5SimAddr,"=pyG5SimPort:",pyG5SimPort)
                    #print("-%%%%- Sending pickled data_dict ... len(serialized_data), data_dict len= ",len(serialized_data), data_dict)                
                except:
                    print("-EEEE-ERROR-: SendAttitudeDataToG5simEfisSim()  ERROR Sending pickle serialized data... ") 
                    print("pyG5SimAddrPort-",pyG5SimAddrPort)
        conn.close

    tmp9= tmp8.replace(": \"" , ": ") #  ' to " keys for pyefis
    #print("tmp9=", tmp9)
    tmp10= tmp9.replace("\"," , ",") #  ", to , keys for pyefis
    #print("tmp10=", tmp10)
    tmp11= tmp10.replace("\"}" , "}") # end quote and curly bracket -> bracket for pyefis
    #print("tmp11=", tmp11)
    #
    
    #if ():
    #
    #heading mag_heading_motors() # add data_dict["hdg"] = heading
    #
    if(len(pyefisSimAddr) != 0):
        data_dict.update(json.loads(tmp11))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as conn:
                try:       
                    serialized_data = pickle.dumps(data_dict)   
                    conn.sendto(serialized_data, (pyefisSimAddr, pyefisSimPort))  #(pyG5SimAddr, pyG5SimPort))  # pyG5SimAddr
                    #msg.get_payload() ) #create byte wise stream from string text   
                    print("--$$$$- Sending pickled data_dict to pyEfis len(serialized_data), data_dict len= " ,len(serialized_data), pyefisSimAddr, "=pyefisSimAddr, ",  pyefisSimPort,"=pyefisSimPort")
                    #print("-$$$$- Sending pickled data_dict ... len(serialized_data), data_dict len= ",len(serialized_data), data_dict)                
                except:
                    print("-EEEE -ERROR-: SendAttitudeDataToG5simEfisSim  ERROR Sending pickle serialized data... ") 
                    print("pyefisSimAddr-",pyefisSimAddr)
        conn.close
    return




################################################################################################
# Init
################################################################################################
# https://dronekit-python.readthedocs.io/en/latest/guide/connecting_vehicle.html
# $ python3 fullmission.py -c 127.0.0.1:14445 connects to jmavsim
# sudo tcpdump -i lo -n udp port 14445 port on jmavsim headless docker 127.0.0.1:14445 default forwarding by qgcs
# $ sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2
# sudo tcpdump -i lo -n udp port 14550 data from sitl at 172.17.0.1
# https://docs.qgroundcontrol.com/master/en/releases/daily_builds.html
# https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl.AppImage
# firmware ardupilot https://firmware.ardupilot.org/Plane/latest/Pixracer/
# https://ardupilot.org/dev/docs/pre-built-binaries.html
# https://firmware.ardupilot.org/Plane/stable/Pixracer-bdshot/arduplane.apj
# https://firmware.ardupilot.org/Plane/stable/
#if(vehicle):
#print("starting sitl autopilot docker x4-gazebo-headless")
#subprocess.Popen(["sudo","docker run --rm -it jonasvautherin/px4-gazebo-headless"]) 
# $ sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2
# cd fixgw
# $ python3 ./mavlink2pyg5.py -c 172.17.0.1:14550
# $ ./fixgw.py -v -d -config-file "fixgw/configs/default.yaml"
# cd pyEfis
# $ sudo python3 ./pyEfis.py
# jf@jfhome:~/pyEfis$ sudo python3 ./pyEfis.py


    #print ("Vehicle connected to sitl/autopilot... start FIX gateway and pyEfis ")
    # from cd ~fixgw.py dir
    #subprocess.Popen(["python3","/home/jf/fixgw/fixgw.py"])
    # " -v -d -config-file /home/jf/fixgw/configs/default.yaml"]) 
    # -config-file fixgw/configs/default.yaml"
    #subprocess.Popen(["python3","./pyG5/pyG5Main.py"]) # non-blocking


# Connect to the Vehicle
# jf@jfhome:~/fixgw$ python3 ./mavlink2pyg5.py -c 172.17.0.1:14550
# $ sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2
# jf@jfhome:~/fixgw$ sudo tcpdump -i lo -n udp port 14550 # from docker gazebo

# open connection to autopilot sitl ardupilot px4
print ("Connecting... to autopilot/sitl")
vehicle = None
# open using connect() dronekit
#vehicle = connect(connection_string, source_system=250, wait_ready=True)
#print (" GPS: %s" % vehicle.gps_0)
#print (" System status: %s" % vehicle.system_status.state)
#print (" Alt: %s" % vehicle.location.global_relative_frame.alt)
#vehicle.close
# open using mavutil.mavlink_connection() reopen connection for mavlink 
vehicle = mavutil.mavlink_connection(connection_string ,source_system=250, wait_ready=True,  ) 
print("Connection made to vehicle, info: (connection_string %s targ.system %u targ.component %u src.sys %u src.comp %u src.vehicle %s" %  (connection_string, vehicle.target_system, vehicle.target_component, vehicle.source_system, vehicle.source_component, vehicle))
# Display basic vehicle state


#$ sudo tcpdump -i lo -n udp port 14445
# $ sudo tcpdump -i docker0 port 14550

# testing connection
'''for x in range(5):
    print(x)
    msg = vehicle.recv_msg() # len(0) if empty
    time.sleep(.05)
    if (msg != None):
        print("-===- recvd mav_msg ", date_time, msg)'''

if(vehicle):
    print("Connected to vehicle, info: (connection_string %s targ.system %u targ.component %u src.sys %u src.comp %u src.vehicle %s" %  (connection_string, vehicle.target_system, vehicle.target_component, vehicle.source_system, vehicle.source_component, vehicle))
    #print (" Type: %s" % vehicle._vehicle_type)
    #print (" Armed: %s" % vehicle.armed)
    #print (" System status: %s" % vehicle.system_status.state)
    #print (" GPS: %s" % vehicle.gps_0)
    #print (" Alt: %s" % vehicle.location.global_relative_frame.alt)

while(vehicle):
    # get mavlink message forever
    # if mavlink type 30 or 33 27 225 226 mavlink attitude or int_gps message send data to pyG5
    # send over tcp udp as 'pickle'd' data dictionary
    #if  vehicle.veh_connected == True  :
    msg = vehicle.recv_msg() 
    time.sleep(.005)
    if (msg != None):
        #if (msg != None): print("msg: ", msg)
        msgID = msg.get_msgId()  
        if (msg != "") and ( (msgID == 30) or (msgID == 33) or (msgID == 141) or (msgID == 27) or (msgID == 225) or (msgID == 226)):
            now = datetime.now()
            date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
            #print("-Received mav_msg, convert, send to G5 & Efis() ", date_time) #, msg)
            print("\r-Received mav_msg, convert, send to G5 & Efis(): ", date_time," ", msg," ", end="")
            SendAttitudeDataToG5simEfisSim(msg)
            # MAVLINK_MSG_ID_ATTITUDE 30
            # MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33
            # MAVLINK_MSG_ID_RAW_IMU 27
            # MAVLINK_MSG_ID_ALTITUDE 141
            # MAVLINK_MSG_ID_RPM 226
    #else:
        #print(".") #error msg empty")
    #time.sleep(.05)   
    

# Disarm vehicle
vehicle.armed = False
time.sleep(1)

# Close vehicle object before exiting script
vehicle.close()
time.sleep(1) 

app.exec()
############################### END #########################




################################################################################################
# Listeners
################################################################################################

#home_position_set = False

#Create a message listener for home position fix
#@vehicle.on_message('HOME_POSITION')
#def listener(self, name, home_position):
#    global home_position_set
#    home_position_set = True


################################################################################################
# Start mission example
################################################################################################

# wait for a home position lock
#while not home_position_set:
#    print ("Waiting for home position...")
#    time.sleep(1)


# do mission
# Change to AUTO mode
PX4setMode(MAV_MODE_AUTO)
time.sleep(1)

# Load commands
cmds = vehicle.commands
cmds.clear()

home = vehicle.location.global_relative_frame

# takeoff to 10 meters
wp = get_location_offset_meters(home, 0, 0, 10);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# move 10 meters north
wp = get_location_offset_meters(wp, 10, 0, 0);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# move 10 meters east
wp = get_location_offset_meters(wp, 0, 10, 0);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# move 10 meters south
wp = get_location_offset_meters(wp, -10, 0, 0);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# move 10 meters west
wp = get_location_offset_meters(wp, 0, -10, 0);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)

# land
wp = get_location_offset_meters(home, 0, 0, 10);
cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
cmds.add(cmd)
print (" UPLOADING")
# Upload mission
cmds.upload()
time.sleep(2)
print (" ARMING")
# Arm vehicle
vehicle.armed = True

# monitor mission execution
nextwaypoint = vehicle.commands.next
print (" START MISSION WAYPOINTS %s" % len(vehicle.commands) )
while nextwaypoint < len(vehicle.commands):
    if vehicle.commands.next > nextwaypoint:
        display_seq = vehicle.commands.next+1
        print ("Moving to waypoint %s" % display_seq)
        nextwaypoint = vehicle.commands.next
    time.sleep(1)
    
print (" LANDING")
# wait for the vehicle to land
while vehicle.commands.next > 0:
    time.sleep(1)







