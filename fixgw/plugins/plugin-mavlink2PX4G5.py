																																																																																		#!/usr/bin/env python

#  Copyright (c) 2014 Phil Birkelbach
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307,
#  USA.import plugin

#  This file serves as a starting point for a plugin.  This is a Thread based
#  plugin where the main Plugin class creates a thread and starts the thread
#  when the plugin's run() function is called.

# Import DroneKit-Python
import time, sys, argparse, math 
import time
from datetime import datetime, timedelta
import socket
import pickle
import subprocess 
import json
import re
from math import *

# plugin
import struct#
import threading
import time
from collections import OrderedDict
import fixgw.plugin as plugin
#
import csv

# timestamp date_time
now = datetime.now()
date_time = now.strftime("%m/%d/%Y, %H:%M:%S")

from platform import python_version
print(python_version())
'''
try:
    # 👇️ using Python 3.10+
    from collections.abc import MutableMapping
except ImportError:
    # 👇️ using Python 3.10-
    from collections     import MutableMapping

# 👇️ <class 'collections.abc.MutableMapping'>
print(MutableMapping)'''


################################################################################################
# Settings
################################################################################################



################################################################################################
# Parse cmd line argument
#parser = argparse.ArgumentParser()
#parser.add_argument("-a", "--address", help="address")
#parser.add_argument("-p", "--port", help="port")
#args = parser.parse_args()

#if args.connect:
    #FIXGWserverIPaddr = args.address
    #FIXGWserverIPport = args.port

#print("FIXGWserverIPaddr:FIXGWserverIPport",FIXGWserverIPaddr ,":",FIXGWserverIPport )

################################################################################################
# jfhome:~/fixgw$ ./fixgw.py -v -d -config-file "fixgw/configs/default.yaml"
# $ sudo tcpdump -i docker0 udp port 1455
# $ sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2
# $ python3 /home/jf/fixgw/mavlink2pyg5.py -c 172.17.0.1:14550
# $ python3 /home/jf/pyEfis/pyEfis.py



class MainThread(threading.Thread):
    def __init__(self, parent):
        global msg
        """The calling object should pass itself as the parent.
           This gives the thread all the plugin goodies that the
           parent has."""
        super(MainThread, self).__init__()
        self.getout            = False   # indicator for when to stop
        self.parent            = parent  # parent plugin object
        self.FIXGWserverIPaddr = parent.config['FIXGWserverIPaddr'] if ('FIXGWserverIPaddr' in parent.config) and parent.config['FIXGWserverIPaddr'] else ''
        self.FIXGWserverIPport = int(parent.config['FIXGWserverIPport']) if ('FIXGWserverIPport' in parent.config) and parent.config['FIXGWserverIPport'] else 3490
        self.pyG5SimAddr       = parent.config['pyG5SimAddr']  
        self.pyG5SimPort       = int(parent.config['pyG5SimPort'])  #65432
        self.pyefisSimAddr     = parent.config['pyefisSimAddr']  #127.0.0.1
        self.pyefisSimPort     = int(parent.config['pyefisSimPort'])  #2100
        self.pyMAVPX4connect   = parent.config['pyMAVPX4connect']  #127.0.0.1:14445 # /dev/ttyUSB0,57600  usb port sik radio  # /dev/ttyACM0,57600  raw usb px connection serial port

        self.timeout = float(parent.config['timeout']) if ('timeout' in parent.config) and parent.config['timeout'] else 1.0
        self.buffer_size = int(parent.config['buffer_size']) if ('buffer_size' in parent.config) and parent.config['buffer_size'] else 1024
        print("starting subprocess mavlink to pyg5 pyefis converter..")
        #print("init() plugin-mavlink2PX4G5  __init__().. self.host self.port self.timeout self.buffer_size ",self.FIXGWserverIPaddr, self.FIXGWserverIPport, self.timeout, self.buffer_size) 
        self.log = parent.log  # simplifies logging
        self.count = 0
        self.keylist = {"ROLL":3, "PITCH":0, "IAS":113, "ALT":4220,
                        "TACH1":2450, "MAP1":24.2, "FUELP1":28.5, "OILP1":56.4,
                        "OILT1":95, "FUELQ1":11.2, "FUELQ2":19.8, "OAT": 32,
                        "CHT11":201,"CHT12":202,"CHT13":199,"CHT14":200,
                        "EGT11":710,"EGT12":700,"EGT13":704,"EGT14":702,
                        "FUELF1":8.7,"VOLT":13.7,"CURRNT":45.6
                        }
        # Initialize the points
        for each in self.keylist:
            self.parent.db_write(each, self.keylist[each])
        # start external apps pyG5 pyEfis mavlink2PX4G5 and sitl if no external pixhawk over sik or direct usb conn
        #command = 'python3 ./mavlink2pyg5.py -m /dev/ttyUSB0,57600 -g 127.0.0.1:65432 -e 127.0.0.1:2100' #pixhawk usb
        #command = 'python3 ./mavlink2pyg5.py -m /dev/ttyACM0,57600 -g 127.0.0.1:65432 -e 127.0.0.1:2100'  #sik radio
        #command = 'python3 ./mavlink2pyg5.py -m 172.17.0.1:14550 -g 127.0.0.1:65432 -e 127.0.0.1:2100' sitl
        #command = 'python3 ./mavlink2PX4G5.py -m 127.0.0.1:14445 -g 127.0.0.1:65432 -e 127.0.0.1:2100' #qgcs forwarded
        # sitl -m 172.17.0.1:14550    # -m 127.0.0.1:14445 qgcs forwarded mav    # PX4 pixhawk direct usb -m /dev/ttyACM0,57600    # sik radio usb /dev/ttyUSB0,57600
        # USEAGE
        # $ python telemetry.py -m "serial:///dev/ttyUSB0:57600" -g 127.0.0.1:65432 -e 127.0.0.1:2100
        # $ python3 FIX-Gateway/mavlink2PX4G5.py -m "/dev/ttyUSB0,57600" -g 127.0.0.1:65432 -e 127.0.0.1:2100
        # $ python telemetry.py -m "udp://:14540" -g 127.0.0.1:65432 -e 127.0.0.1:2100
        if(self.pyMAVPX4connect == "172.17.0.1:14550") or (self.pyMAVPX4connect == "udp://:14540") or \
          (self.pyMAVPX4connect == "udp:127.0.0.1:14445") or(self.pyMAVPX4connect == "udp:172.17.0.1:14540")or(self.pyMAVPX4connect == "udp://172.17.0.1:14550") or \
          (self.pyMAVPX4connect == "udp://:14445") or (self.pyMAVPX4connect == "udp://:14550"): # udp://:14550 # launch sitl if connection string matches "172.17.0.1:14550"
            print("$$$-starting PX4 SIM subprocess sitl..  $ sudo docker run --rm -it --env PX4_HOME_LAT=42.397742 --env PX4_HOME_LON=-79.1 --env PX4_HOME_ALT=488.0 jonasvautherin/px4-gazebo-headless:1.14.0") #" #"sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2")
            #command = 'sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2' 
            #command = 'sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2'  
            command = 'sudo docker run --rm -it --env PX4_HOME_LAT=42.397742 --env PX4_HOME_LON=-79.1 --env PX4_HOME_ALT=488.0 jonasvautherin/px4-gazebo-headless:1.14.0'
            subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
            time.sleep(5) # let sitl startup before connecting
        #
        #command = 'python3 ./mavlink2PX4G5.py -m '+self.pyMAVPX4connect+' -g '+self.pyG5SimAddr+':'+str(self.pyG5SimPort)+' -e '+ self.pyefisSimAddr+':'+str(self.pyefisSimPort)
        # mavlinkMAVSDKdronekitCombinedJan13joystickManCtl.p
        #command = 'python ./mavlinkMAVSDKdronekitCombinedJan13joystickManCtl.py -m '+self.pyMAVPX4connect+' -g '+self.pyG5SimAddr+':'+str(self.pyG5SimPort)+' -e '+ self.pyefisSimAddr+':'+str(self.pyefisSimPort)
        command = 'python3 ./mavlinkMAVSDKdronekitCombined.py -m '+self.pyMAVPX4connect+' -g '+self.pyG5SimAddr+':'+str(self.pyG5SimPort)+' -e '+ self.pyefisSimAddr+':'+str(self.pyefisSimPort)
        # /home/jf/testgit/FIX-Gateway/mavlinkMAVSDKdronekitCombined.py
        print("$$$-starting command /home/jf/testgit/FIX-Gateway/mavlinkMAVSDKdronekitCombined.py:",command)
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        #  gnome-terminal --geometry=120x20+500+500     --geometry 40x80+100+100 didnt work right

        print("$$$-starting subprocess pyEfis ../pyEfis/pyEfis.py")
        command = 'python3 ../pyEfis/pyEfis.py' 
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        #
        #python3 /home/jf/pyg5/Scripts/pyG5DualStacked 
        print("$$$-starting subprocess pyG5 ../pyG5/pyG5/pyG5Main.py -m hsi")
        command = 'python3 ../pyG5/pyG5/pyG5Main.py -m hsi' # hsi is smaller wondow full is fullscreen python3 /home/jf/pyG5-main/pyG5/pyG5Main.py'   
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command], stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        #
        
        #
        # $ sudo tcpdump -i docker0 udp port 14550 
        #
        # jf@jfhome:~/fixgw$ ./fixgw.py -v -d -config-file "fixgw/configs/default.yaml"
        '''
        # default.yaml plugin for pyG5 and pyEfis conversion data fom mavlink msgs
          mavlink2PX4G5:
           load:              yes
           module:            fixgw.plugins.plugin-mavlink2PX4G5
           FIXGWserverIPaddr: 127.0.0.1
           FIXGWserverIPport: 2100
           pyG5SimAddr:       127.0.0.1
           pyG5SimPort:       65432
           pyefisSimAddr:     127.0.0.1
           pyefisSimPort:     2100
           pyMAVPX4connect:   127.0.0.1:14445 # /dev/ttyUSB0,57600  usb port sik radio  # /dev/ttyACM0,57600  raw usb px connection serial port
           buffer_size:       1024
           timeout:           10.0 '''

    def run(self):
        global msg
        
        while True:
            if self.getout:
                break
            time.sleep(.05) # loop delay
            self.count += 1
            self.tmp="FIXGW(run) mavlink plugin run loop thread... waiting for data on ... "+self.FIXGWserverIPaddr+" self.FIXGWserverIPaddr ,  "+ str(self.FIXGWserverIPport)+"self.FIXGWserverIPport"
            #self.log.debug(self.tmp) 
            # Do something useful here  
            self.running = False
            #print("self.FIXGWserverIPaddr-", self.FIXGWserverIPaddr ,  "  self.FIXGWserverIPport-",self.FIXGWserverIPport)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as self.conn: # open conn
                #self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.conn.bind((self.FIXGWserverIPaddr, self.FIXGWserverIPport))
                #self.s.listen()  
                #self.conn, self.addr = self.s.accept()
                with self.conn:
                    #print(f"Connected to {self.addr}")
                    self.data_pickled = self.conn.recv(1024)
                    self.data_dict = pickle.loads(self.data_pickled)   
                    print("FIXGW server plugin rxd mavlink_data_dict:  ",self.data_dict, flush=False, end='\r')
                    #print("\r-FIXGW.run()-Received pickled mav_msg pickled-len()", date_time," ", len(self.data_pickled)) #, self.data_dict)
                    # copy key, value pairs into data dict for efis                   
                    #try: self.parent.db_write("ALT", self.data_dict["alt"]/1000)
                    #except: pass
                    
                    # internal KEY = Data pack values form pickle data into internal dict for pyefis 
                    # for MAVSDK recv msgs
                    #
                    try: 
                        self.parent.db_write("ALT",    self.data_dict["relative_altitude_m"]*3.28084) # convert to feet from meters #*100)   # "absolute_altitude_m"]*100)
                        #print("\nALT mavsdk", self.data_dict["relative_altitude_m"]) 
                    except: 
                        #print("error writing key val ALT mavsdk", self.data_dict["relative_altitude_m"]) 
                        pass
                    try: 
                        self.parent.db_write("PITCH", self.data_dict["pitch_deg"]) 
                        #print("\nPITCH mavsdk")
                    except: pass
                    try: 
                        self.parent.db_write("ROLL",  self.data_dict["roll_deg"])
                        #print("\nROLL mavsdk")
                    except: pass
                    try: 
                        self.parent.db_write("YAW",   self.data_dict["yaw_deg"]) 
                        #print("\nYAW mavsdk")
                    except: pass
                    try: 
                        self.parent.db_write("LAT",   self.data_dict["latitude_deg"]) 
                        #print("\nLAT mavsdk")
                    except: pass
                    try: 
                        self.parent.db_write("LONG",  self.data_dict["longitude_deg"]) 
                        #print("\nLONG mavsdk")                        
                    except: pass
                    try: 
                        self.parent.db_write("HEAD",   self.data_dict["heading_deg"]) 
                        #print("\nHEAD mavsdk")
                    except: pass
                    # for dronekit pymavlink data converter
                    try: # and fail no keys written
                        self.parent.db_write("ALT",    self.data_dict["alt"])
                        #print("\nALT dronekit")
                    except: pass
                    try: 
                        self.parent.db_write("PITCH", self.data_dict["pitch"]*100) 
                        #print("\nPITCH dronekit")
                    except: pass
                    try: 
                        self.parent.db_write("ROLL",  self.data_dict["roll"]*100)
                        #print("\nROLL dronekit")
                    except: pass
                    try: 
                        self.parent.db_write("YAW",   self.data_dict["yaw"]) 
                        #print("\nYAW dronekit")
                    except: pass
                    try: 
                        self.parent.db_write("LAT",   self.data_dict["lat"]) 
                        #print("\nLAT dronekit")
                    except: pass
                    try: 
                        self.parent.db_write("LONG",  self.data_dict["lon"]) 
                        #print("\nLONG dronekit")
                    except: pass
                    try: 
                        self.parent.db_write("HEAD",   self.data_dict["hdg"]/100) 
                        #print("\nHEAD dronekit")
                    except: pass
                    #
                    
                self.conn.close()
        '''import csv example
        Creating list of field names
        field_names= ['No', 'Company', 'Car Model']
        Creating a list of python dictionaries

        cars = [
        {‘No’: 1, ‘Company’: ‘Ferrari’, ‘Car Model’: ‘488 GTB’},
        {‘No’: 2, ‘Company’: ‘Porsche’, ‘Car Model’: ‘918 Spyder’},
        {‘No’: 3, ‘Company’: ‘Bugatti’, ‘Car Model’: ‘La Voiture Noire’},
        {‘No’: 4, ‘Company’: ‘Rolls Royce’, ‘Car Model’: ‘Phantom’},
        {‘No’: 5, ‘Company’: ‘BMW’, ‘Car Model’: ‘BMW X7’},
        ] 
 
        Writing content of dictionaries to CSV file

        with open('Names.csv', 'w') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=field_names)
        writer.writeheader()
        writer.writerows(cars)'''
        
    def stop(self):
        self.getout = True


class Plugin(plugin.PluginBase):
    """ All plugins for FIX Gateway should implement at least the class
    named 'Plugin.'  They should be derived from the base class in
    the plugin module.

    The run and stop methods of the plugin should be overridden but the
    base module functions should be called first."""
    def __init__(self, name, config):
        super(Plugin, self).__init__(name, config)
        self.thread = MainThread(self)

    def run(self):
        """ The run method should return immediately.  The main routine will
        block when calling this function.  If the plugin is simply a collection
        of callback functions, those can be setup here and no thread will be
        necessary"""
        self.thread.start()

    def stop(self):
        """ The stop method should not return until the plugin has completely
        stopped.  This generally means a .join() on a thread.  It should
        also undo any callbacks that were set up in the run() method"""
        self.thread.stop()
        if self.thread.is_alive():
            self.thread.join(1.0)
        if self.thread.is_alive():
            raise plugin.PluginFail

    def get_status(self):
        """ The get_status method should return a dict or OrderedDict that
        is basically a key/value pair of statistics"""
        return OrderedDict({"Count":self.thread.count})
