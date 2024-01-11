import pygame
import time

pygame.display.init() #? required
pygame.joystick.init()

done = False
OldAxis = []
Axis = []
joysticks={}
PrevJoystickVals = {} # store prev vals
#JoyAxis =[]

# start up package
# Get count of joysticks.
joystick_count = pygame.joystick.get_count()
print(f"Number of joysticks: {joystick_count}")


JoyAxes = joystick_count   #pygame.joystick.Joystick(0).get_numaxes()
#print ("Number of axis:", JoyAxes)

# init each joystick object in pygame
#pygame.joystick.Joystick(0).init()
# Prints the joystick's name
#JoyName = pygame.joystick.Joystick(0).get_name()
#print ("Name of the joystick:", JoyName)
# Gets the number of axes

#
if(joystick_count > 0 ):
    print ("Number of joysticks:", JoyAxes)
    done = False
    while not done:
        #
        # Event processing step.
        # Possible joystick events: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
        # JOYBUTTONUP, JOYHATMOTION, JOYDEVICEADDED, JOYDEVICEREMOVED
        # and then for each joystick create a dict entry using guid as key and object addr and guid and name etc as values as a list
        for i, j in PrevJoystickVals.items():
            if PrevJoystickVals[i] != joysticks[i]:
                print ("joystick event:",str(i)," vals:",j)
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
                    print ("NEW joystick dict:",str(i)," vals:",j)
                #joysticks[guid] = [joy] # causes error dictionary resized during iteration
                print(f"Joystick {joy.get_instance_id()} is now connected {joysticks.keys()} ; {joysticks.values()} ; {joysticks.items()}" )
                joystick_count = pygame.joystick.get_count()
                # init each joystick object in pygame
                #pygame.joystick.Joystick(event.device_index).init()

            if event.type == pygame.JOYDEVICEREMOVED:
                del joysticks[event.instance_id]
                print(f"Joystick {event.instance_id} is now disconnected")
                print("joysticks{}: joysticks.values()",joysticks,joysticks.values())
                # remove joy object from joysticks dicst
                #joy = pygame.joystick.Joystick(event.device_index)
                #del joysticks[joy.get_instance_id()]
        #
        # For each joystick:
        #print(f"# of joysticks: {joystick_count}  {joysticks} ")
        #
        #if (len(joysticks)==0):
        #print("len(joysticks)==0 joysticks.values()",joysticks.values())
        #time.sleep(.050)
        #pygame.event.pump()
        for joystick in joysticks.values():
            #jid = joystick[0].get_instance_id()
            guid = joystick[0].get_guid() # from PYTHON OBJECT addr get guid from above when added new joystick event
            #jid = joystick[0].get_id()
            #print(f"Joystick jid {jid}")
            #joysticks[guid] = [ joystick[0] ]
            #print("joysticks[jid]:",joysticks[jid])
            joysticks.update( { guid : [ joystick[0] ] } ) # restart remove vals for key and replace with guid
            # Get the name from the OS for the controller/joystick.
            name = joystick[0].get_name()
            #print(f"Joystick name: {name}")
            joysticks[guid].append(name)
            #print("add name joysticks[jid]:",joysticks[jid])

            #guid = joystick[0].get_guid()
            #print(f"GUID: {guid}")
            #joysticks[guid].append(guid)
            #print("add GUID joysticks[jid]:",joysticks[jid])

            power_level = joystick[0].get_power_level()
            #print(f"Joystick's power level: {power_level}")
            joysticks[guid].append(power_level)
            #print("add power_level joysticks[jid]:",joysticks[jid])

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
                joysticks[guid].append(hat)
                #print(f"Hat {i} value: {str(hat)}")
                #print(f"add hat {i} {joysticks[jid]}")
            #===============
        #print(f"{joysticks[jid]}") # print entire list for joystick for the joy id
        #for i, j in joysticks.items():
        #    joysticks[jid] = [ joystick[0] ]
        #    print ("joystick:",str(i)," vals:",j)
       

