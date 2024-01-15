#!/usr/bin/env python3

"""
This example shows how to use the manual controls plugin.

Note: Manual inputs are taken from a test set in this example to decrease
complexity. Manual inputs can be received from devices such as a joystick
using third-party python extensions.

Note: Taking off the drone is not necessary before enabling manual inputs.
It is acceptable to send positive throttle input to leave the ground.
Takeoff is used in this example to decrease complexity
"""

import asyncio
import random
from mavsdk import System
import time

import keyboard 


import pygame

# timestamp date_time
from datetime import datetime, timedelta
from mavsdk import System
### INIT global vars etc
pygame.display.init() #? required
pygame.joystick.init()

Axis = []
joysticks={}
PrevJoystickVals = {} # store prev vals
done = False 

#################### FUNCTIONS

async def manual_controls():
    """Main function to connect to the drone and input manual controls"""
    # Connect to the Simulation
    drone = System()
    print("Waiting for drone to connect...")
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

    # Arming the drone
    print("-- Arming")
    await drone.action.arm()

    # Takeoff the vehicle
    print("-- Taking off")
    await drone.action.takeoff()
    time.sleep(15) # wait to get to position hold

    print("-- Starting manual control")
    await drone.manual_control.set_manual_control_input(float(0), float(0), float(0.5), float(0)) #needs this ?
    
    # send mavlink messages from usb josticks
    asyncio.ensure_future(joystick_event(drone))
    #asyncio.ensure_future(print_position(drone))
    #asyncio.ensure_future(print_attitude_euler(drone))
    #asyncio.ensure_future(print_heading(drone))
    while True:
        await asyncio.sleep(1)

async def joystick_event(drone):
    global roll, pitch, throttle, yaw, PrevJoystickVals
    print("--MAVSDK--async joystick_event()")
    while(True):
        #if(True):  #joystick_count > 0 ):
        # Get count of joysticks.
        joystick_count = pygame.joystick.get_count()
        JoyAxes = joystick_count   #pygame.joystick.Joystick(0).get_numaxes()
        print(f"Number of joysticks: {joystick_count}")
        done = False
        while not done:
            #
            # Event processing step.# Possible joystick events: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
            # JOYBUTTONUP, JOYHATMOTION, JOYDEVICEADDED, JOYDEVICEREMOVED
            # and for each joystick create a dict entry using guid as key and object addr and guid and name etc as values as a list
            dec = 3 # round to decimal
            for i, j in PrevJoystickVals.items():
                if (True):
                    now = datetime.now()
                    date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
                    print(f"--MAVSDK--async joystick_event(): date:{date_time}, guid:{i}, roll:{j[5]}, pitch:{j[6]}, throttle:{j[7]},  yaw:{j[9]}" )
                    await drone.manual_control.set_manual_control_input( float(j[5]), float(j[6]), float(j[7]), float(j[9]) )
                    #only if chnaged below
                    #if  (round(float(PrevJoystickVals[i][5]), dec) != round(float(joysticks[i][5]), dec)) or \
                    #(round(float(PrevJoystickVals[i][6]), dec) != round(float(joysticks[i][6]), dec)) or \
                    #(round(float(PrevJoystickVals[i][7]), dec) != round(float(joysticks[i][7]), dec)) or \
                    #(round(float(PrevJoystickVals[i][8]), dec) != round(float(joysticks[i][8]), dec)) or \
                    #(round(float(PrevJoystickVals[i][9]), dec) != round(float(joysticks[i][9]), dec)) or \
                    #(round(float(PrevJoystickVals[i][10]), dec) != round(float(joysticks[i][10]), dec) ):'''
 
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

                # Handle hotplugging (adding removing in real time usb joysticks
                if event.type == pygame.JOYDEVICEADDED:
                    # This event will be generated when the program starts for every
                    # joystick, filling up the list without needing to create them manually.
                    joy = pygame.joystick.Joystick(event.device_index)
                    joysticks.update({joy.get_guid():[joy]}) # add object addr val to key=guid dict joysticks
                    print ("ADDING NEW joystick key:",joy.get_guid(),"  val:",[joy])
                    for i, j in joysticks.items():
                        print ("ADDED NEW joystick dict:",str(i)," vals:",j)
                    print(f"ADDED Joystick {joy.get_instance_id()} is now connected {joysticks.keys()} ; {joysticks.values()} ; {joysticks.items()}" )
                    joystick_count = pygame.joystick.get_count()

                if event.type == pygame.JOYDEVICEREMOVED:
                    for k, v in joysticks.items(): #for joystick in joysticks:  #.values():   #for joystick in joysticks:
                        jid = v[1] #.get_instance_id()
                        guid = k # keys are guid from pygame joy.get_guid()
                        print ("DISCONNECTING guid:",guid,"  jid:",jid)
                        if (event.instance_id == jid): 
                            print ("DEL joysticks[guid] dict guid, inst id jid:",guid, jid)
                            print(f"DISCONNECT Joystick {event.instance_id} is now disconnected")
                            del joysticks[k] #event.instance_id] # joy.get_guid()
                            PrevJoystickVals = {} 
                            break # short circuit
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

        print("running jmanual control oystick loop ...") #"drone.manual_control.set_manual_control_input(pitch, roll, throttle, yaw)")
        await asyncio.sleep(0.1)


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(manual_controls())
