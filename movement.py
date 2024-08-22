from dronekit import connect, Command,VehicleMode,LocationGlobalRelative
from pymavlink import mavutil
import time

# flag
i = False

# Arm and takeoff function
def arm_and_takeoff(vehicle,aTargetAltitude):
    print("Basic pre-arm checks")

    # Checking if the drone is ready to takeoff
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED") # Copter should arm in GUIDED mode
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

# RTL
def return_to_launch(vehicle):
    print('Return to launch')
    vehicle.mode = VehicleMode("RTL")
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt<=0.3: # checking for landing altitude
            print("Landing Successful")
            break
        time.sleep(1)

# mission execution:
def mission_exe(vehicle,mission_lst,event):
    global i
    vehicle.mode = VehicleMode("AUTO") # runing mission in "auto" mode
    for i in range(0,len(mission_lst)-1): # (0,len(mission_lst)-1)
        event.wait()
        wp = vehicle.commands[i]
        print(wp)
        lat = wp.x
        lon = wp.y
        alt = wp.z
        targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
        vehicle.simple_goto(targetWaypointLocation)
        #time.sleep(10)
        i = True
        event.clear()

def waypoint_reached():
    global i
    if i:
        i = False
        return True
    else:
        return False

def decent_10(vehicle):
    location = vehicle.location.global_relative_frame
    lat = location.lat
    lon = location.lon
    target_decent = LocationGlobalRelative(lat,lon,10)
    vehicle.simple_goto(target_decent)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt<=11: # checking for landing altitude
            print("Decent_10 Successfull")
            break
        time.sleep(2)

def decent_20(vehicle):
    location = vehicle.location.global_relative_frame
    lat = location.lat
    lon = location.lon
    target_decent = LocationGlobalRelative(lat,lon,20)
    vehicle.simple_goto(target_decent)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt<=21: # checking for landing altitude
            print("Decent_20 Successfull")
            break
        time.sleep(2)

def increase_alt_30(vehicle):
    location = vehicle.location.global_relative_frame
    lat = location.lat
    lon = location.lon
    target_decent = LocationGlobalRelative(lat,lon,30)
    vehicle.simple_goto(target_decent)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=29: # checking for landing altitude
            print("increment 30 Successfull")
            break
        time.sleep(2)

def forward(vehicle,pos_x):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        int(0b110111111000), # type_mask
        pos_x, 0, 0, # x, y, z positions
        0, 0, 0, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
    # for x in range(0,duration):
    #     vehicle.send_mavlink(msg)
    #     time.sleep(1)
    vehicle.send_mavlink(msg)
    time.sleep(8)

def backward(vehicle,pos_x):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        int(0b110111111000), # type_mask
        -pos_x, 0, 0, # x, y, z positions
        0, 0, 0, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
    # for x in range(0,duration):
    #     vehicle.send_mavlink(msg)
    #     time.sleep(1)
    vehicle.send_mavlink(msg)
    time.sleep(8)

def right(vehicle,pos_y):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        int(0b110111111000), # type_mask
        0, pos_y, 0, # x, y, z positions
        0, 0, 0, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
    # for x in range(0,duration):
    #     vehicle.send_mavlink(msg)
    #     time.sleep(1)
    vehicle.send_mavlink(msg)
    time.sleep(8)

def left(vehicle,pos_y):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        int(0b110111111000), # type_mask
        0, -pos_y, 0, # x, y, z positions
        0, 0, 0, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
    # for x in range(0,duration):
    #     vehicle.send_mavlink(msg)
    #     time.sleep(1)
    vehicle.send_mavlink(msg)
    time.sleep(8)

def diagonal_mov(vehicle,pos_x,pos_y):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        int(0b110111111000), # type_mask
        pos_x, pos_y, 0, # x, y, z positions
        0, 0, 0, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
    # for x in range(0,duration):
    #     vehicle.send_mavlink(msg)
    #     time.sleep(1)
    vehicle.send_mavlink(msg)
    time.sleep(8)

def controlServo(vehicle,servo_number,pwm_value):
    msg = vehicle.message_factory.command_long_encode(
            0,
            0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_number,  # we use channel 8
            pwm_value,     # 1100
            0,
            0,
            0,
            0,
            0)
    vehicle.send_mavlink(msg)
    time.sleep(3)