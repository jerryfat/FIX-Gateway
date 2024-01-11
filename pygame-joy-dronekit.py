import pygame

# timestamp date_time
from datetime import datetime, timedelta
from mavsdk import System

pygame.display.init() #? required
pygame.joystick.init()

Axis = []
joysticks={}
PrevJoystickVals = {} # store prev vals
done = False 
#
# start up package
# Get count of joysticks.
'''joystick_count = pygame.joystick.get_count()
JoyAxes = joystick_count   #pygame.joystick.Joystick(0).get_numaxes()
print(f"Number of joysticks: {joystick_count}")
'''
#  --connect "udp://172.17.0.1:14540"
import argparse  
parser = argparse.ArgumentParser(description='Example showing how to set and clear vehicle channel-override information.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
#
# for dronekit
'''# changes made to /usr/lib/python3.10/collections/__init__.py
from _collections_abc import MutableMapping 
import _collections_abc 
#import _collections_abc # orig'''

# Import DroneKit-Python
from dronekit import connect, Command, LocationGlobal # from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command   
#vehicle = mavutil.mavlink_connection(connection_string ,source_system=250, wait_ready=True,  ) 

from pymavlink import mavutil
#vehicle.connect(system_address="udp://:14540") 

sitl = None
vehicle = None


# try to connect
#print('Connecting to vehicle on: %s' % connection_string)

# Connect to the Vehicle
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
#vehicle = connect(connection_string, wait_ready=True)
#dronekit  
vehicle = mavutil.mavlink_connection(connection_string ,source_system=250, wait_ready=True,  ) 
# Connect to the Simulation
# mavsdk vehicle = System()
# mavsdk vehicle.connect("udp://:14445, wait_ready=True") #vehicle.connect(system_address=connection_string) #"udp://:14540") 
if(vehicle): 
    print("Connection made to vehicle")
    #print("Connection made to vehicle, info: (connection_string %s targ.system %u targ.component %u src.sys %u src.comp %u src.vehicle %s" %  \
    #  (connection_string, vehicle.target_system, vehicle.target_component, vehicle.source_system, vehicle.source_component, vehicle))
    # Get all original channel values (before override)
    #print("Number of RC channels: %s" % len(vehicle.channels))
    #print("Channel values from RC Tx:", vehicle.channels)
else:
    print("error connecting, quitting app...")
    exit()
#
#
if(True):  #joystick_count > 0 ):
    # Get count of joysticks.
    joystick_count = pygame.joystick.get_count()
    JoyAxes = joystick_count   #pygame.joystick.Joystick(0).get_numaxes()
    print(f"Number of joysticks: {joystick_count}")
    done = False
    while not done:
        #
        # Event processing step.
        # Possible joystick events: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
        # JOYBUTTONUP, JOYHATMOTION, JOYDEVICEADDED, JOYDEVICEREMOVED
        # and then for each joystick create a dict entry using guid as key and object addr and guid and name etc as values as a list
        dec = 3 # round to decimal
        for i, j in PrevJoystickVals.items():
            if  (round(float(PrevJoystickVals[i][5]), dec) != round(float(joysticks[i][5]), dec)) or \
                (round(float(PrevJoystickVals[i][6]), dec) != round(float(joysticks[i][6]), dec)) or \
                (round(float(PrevJoystickVals[i][7]), dec) != round(float(joysticks[i][7]), dec)) or \
                (round(float(PrevJoystickVals[i][8]), dec) != round(float(joysticks[i][8]), dec)) or \
                (round(float(PrevJoystickVals[i][9]), dec) != round(float(joysticks[i][9]), dec)) or \
                (round(float(PrevJoystickVals[i][10]), dec) != round(float(joysticks[i][10]), dec) ):
                #now = datetime.now()
                #date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
                #print (date_time, " joystick event:",str(i)," vals:",j)
                # send manual control or channel overide to mavlink here
                if(vehicle): print("connected...")
                #  {j[5]} {j[6]} {j[7]} {j[9]} AETR on great planes i controller rc usb device
                print(f"{i} {j[5]} {j[6]} {j[7]} {j[8]} {j[9]} {j[10]}")
                ail = 0 #int(round((j[5]*1000),0))
                ele = 0 #int(round((j[6]*1000),0))
                thr = 0 #int(round((j[7]*1000),0))
                rud = 1 #int(round((j[9]*1000),0)) 
                buttons = 1 + 1 << 3 + 1 << 7
                print (f"vehicle.mav.manual_control_send sysid:{vehicle.source_system} ail:{ail} ele:{ele} thr:{thr} rud:{rud} but:{buttons} " )
                vehicle.mav.manual_control_send(vehicle.source_system, ail , ele , thr , rud, buttons)
                #vehicle.manual_control.set_manual_control_input(j[5],j[6],j[7],j[9])  #drone.manual_control.set_manual_control_input(pitch, roll, throttle, yaw)
                #    print("Connection made to vehicle, info: (connection_string %s targ.system %u targ.component %u src.sys %u src.comp %u src.vehicle %s" %  (connection_string, vehicle.target_system, vehicle.target_component, vehicle.source_system, vehicle.source_component, vehicle))
                    # 
 
        # check if current vals different tahn prev vals
        PrevJoystickVals.clear() # clears prev vals in prev vals dict
        for i, j in joysticks.items():
            #print ("joystick:",str(i)," vals:",j)
            PrevJoystickVals[i] =  j # add current vals into old vals before rewrting new vals from joystick
        #
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True  # Flag that we are done so we exit this loop.

            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
                #if event.button == 0:
                    #joystick = joysticks[event.device_index] #instance_id]
                    #if joystick.rumble(0, 0.7, 500):
                        #print(f"Rumble effect played on joystick {event.instance_id}")

            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")

            # Handle hotplugging
            if event.type == pygame.JOYDEVICEADDED:
                # This event will be generated when the program starts for every
                # joystick, filling up the list without needing to create them manually.
                joy = pygame.joystick.Joystick(event.device_index)
                #guid = joy.get_guid()
                #joysticks[joy.get_guid()] = [joy] #( key:index val is list with [0] object being object id )
                # update the dictionary with the author key-value pair
                #site.update({'Author':'Sammy Shark'})
                joysticks.update({joy.get_guid():[joy]}) # add object addr val to key=guid dict joysticks
                print ("ADDING NEW joystick key:",joy.get_guid(),"  val:",[joy])
                for i, j in joysticks.items():
                    print ("ADDED NEW joystick dict:",str(i)," vals:",j)
                #joysticks[guid] = [joy] # causes error dictionary resized during iteration
                print(f"ADDED Joystick {joy.get_instance_id()} is now connected {joysticks.keys()} ; {joysticks.values()} ; {joysticks.items()}" )
                joystick_count = pygame.joystick.get_count()
                # init each joystick object in pygame
                #pygame.joystick.Joystick(event.device_index).init()

            if event.type == pygame.JOYDEVICEREMOVED:
                for k, v in joysticks.items(): #for joystick in joysticks:  #.values():   #for joystick in joysticks:
                    jid = v[1] #.get_instance_id()
                    guid = k # keys are guid from pygame joy.get_guid()
                    #guid = joystick[0].get_guid()
                    print ("DISCONNECTING guid:",guid,"  jid:",jid)
                    if (event.instance_id == jid): 
                        #joy = pygame.joystick.Joystick(event.device_index)
                        #del joysticks[joy.get_instance_id()]
                        #joy = pygame.joystick.Joystick(event.instance_id)
                        print ("DEL joysticks[guid] dict guid, inst id jid:",guid, jid)
                        print(f"DISCONNECT Joystick {event.instance_id} is now disconnected")
                        del joysticks[k] #event.instance_id] # joy.get_guid()
                        PrevJoystickVals = {} 
                        break # short circuit
                   # print( "DISCONNECT dict joysticks=",joysticks) #,"  joysticks.values()",joysticks.values())
                # remove joy object from joysticks dicst
                #joy = pygame.joystick.Joystick(event.device_index)
                #del joysticks[joy.get_instance_id()]
        #
        for joystick in joysticks.values():
            jid = joystick[0].get_instance_id()
            guid = joystick[0].get_guid() # from PYTHON OBJECT addr get guid from above when added new joystick event
            joysticks.update( { guid : [ joystick[0] ] } ) # restart remove vals for key and replace with guid
            joysticks[guid].append(jid)
            # Get the name from the OS for the controller/joystick.
            name = joystick[0].get_name() #print(f"Joystick name: {name}")
            joysticks[guid].append(name) #print("add name joysticks[jid]:",joysticks[jid])

            #guid = joystick[0].get_guid()
            #print(f"GUID: {guid}")
            #joysticks[guid].append(guid)
            #print("add GUID joysticks[jid]:",joysticks[jid])

            power_level = joystick[0].get_power_level()  #print(f"Joystick's power level: {power_level}")
            joysticks[guid].append(power_level)  #print("add power_level joysticks[jid]:",joysticks[jid])

            # Usually axis run in pairs, up/down for one, and left/right for
            # the other. Triggers count as axes.
            axes = joystick[0].get_numaxes()
            #print(f"Number of axes: {axes}")
            joysticks[guid].append(axes)
            #print("add axes joysticks[jid]:",joysticks[jid])

            for i in range(axes):
                axis = joystick[0].get_axis(i)
                #print(f"Axis {i} value: {axis:>6.3f}")
                joysticks[guid].append(round(axis,5))
                #print(f"add axis {i} {joysticks[jid]}")

            buttons = joystick[0].get_numbuttons()
            #print(f"Number of buttons: {buttons}")
            joysticks[guid].append(buttons)
            #print(f"add buttons {buttons} {joysticks[jid]}")

            for i in range(buttons):
                button = joystick[0].get_button(i)
                #print(f"Button {i:>2} value: {button}")
                joysticks[guid].append(button)
                #print(f"add button {i} {joysticks[jid]}")

            hats = joystick[0].get_numhats()
            #print(f"Number of hats: {hats}")
            joysticks[guid].append(hats)
            #print(f"add hats {hats} {joysticks[jid]}")

            # Hat position. All or nothing for direction, not a float like
            # get_axis(). Position is a tuple of int values (x, y).
            for i in range(hats):
                hat = joystick[0].get_hat(i)
                #print(f"hat: {hat}")
                joysticks[guid].append(hat) # comes back as a list, not an int value
                #print(f"Hat {i} value: {str(hat)}")
                #print(f"add hat {i} {joysticks[jid]}")

################# END #################### END #####################
''' # had to change line in:
/usr/lib/python3.10/collections/__init__.py
/home/jf/.local/lib/python3.10/site-packages/dronekit/__init__.py
#import _collections_abc
to:
# import _collections_abc and replace with: 
try: # for dronekit after python 3.8 
    #using Python 3.10+ 
    import _collections_abc
    #import MutableMapping 
except ImportError: 
    #using Python 3.10- 
    import _collections 
'''

