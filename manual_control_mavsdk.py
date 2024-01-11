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
#kp.init()

# Test set of manual inputs. Format: [roll, pitch, throttle, yaw]
manual_inputs = [
    [0, 0, 0.5, 0],  # no movement
    [-1, 0, 0.5, 0],  # minimum roll
    [1, 0, 0.5, 0],  # maximum roll
    [0, -1, 0.5, 0],  # minimum pitch
    [0, 1, 0.5, 0],  # maximum pitch
    [0, 0, 0.5, -1],  # minimum yaw
    [0, 0, 0.5, 1],  # maximum yaw
    [0, 0, 1, 0],  # max throttle
    [0, 0, 0, 0],  # minimum throttle
]


async def manual_controls():
    """Main function to connect to the drone and input manual controls"""
    # Connect to the Simulation
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Get all original channel values (before override)
    # nope print("Channel values from RC Tx:", drone.channels) 
    # nope print( drone.recv_match(type='RC_CHANNELS', blocking=True) )

    # This waits till a mavlink based drone is connected
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    # Checking if Global Position Estimate is ok
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # Arming the drone
    print("-- Arming")
    await drone.action.arm()

    # Takeoff the vehicle
    print("-- Taking off")
    await drone.action.takeoff()
    time.sleep(15)

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    #while True:
    # kp.init() # init keyboard events
    # connect via mavsdk()
    # Init the drone
    #drone = System()
    #print("libs Connecting... to autopilot/sitl")
    # px4 sitl sudo docker run --rm -it jonasvautherin/px4-gazebo-headless:1.13.2
    #await drone.connect(Px4simAddrPort) #connection_string) # "udp://:14540") # yup 
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

        # set the manual control input after arming and takeo

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
    #asyncio.ensure_future(print_position(drone))
    #asyncio.ensure_future(print_attitude_euler(drone))
    #asyncio.ensure_future(print_heading(drone))
    while True:
        await asyncio.sleep(1)

async def getKeyboardInput(my_drone):
    global roll, pitch, throttle, yaw
    while True:
        roll, pitch, throttle, yaw = 0, 0, 0.5, 0
        value = 1
        # Wait for the next event.
        event = keyboard.read_event()
        if event.event_type == keyboard.KEY_DOWN and event.name == 'space':
            print('space was pressed')
            print("event:", event.name)
            event = keyboard.read_event()
        '''if kp.getKey("LEFT"):
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
        '''
        print("getKeyboardInput:roll, pitch, throttle, yaw",roll, pitch, throttle, yaw)
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
    while(True):
        #await drone.manual_control.set_manual_control_input(pitch, roll, throttle, yaw)
        # if keyboard or joystick event then send to drone
        print("running joystick manual control loop ...") #"drone.manual_control.set_manual_control_input(pitch, roll, throttle, yaw)")
        await asyncio.sleep(0.1)


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(manual_controls())
