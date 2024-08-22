#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronekit import connect, Command,VehicleMode,LocationGlobalRelative
from pymavlink import mavutil
import time
import threading
import argparse  
from mission import *
from movement import *
from cv import *


# FLags
hotspot_flag = False
target_flag = False
prev_found = False
target_not_found = True
loop = False
i = 1
count_h = 0
cmd_id = -1

# connection as argument:
parser = argparse.ArgumentParser(description='Connection with drone')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = args.connect

# Start SITL if no connection string specified
# sitl = None
# if not connection_string:
#     import dronekit_sitl
#     sitl = dronekit_sitl.start_default()
#     connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

# Ground Speed
vehicle.groundspeed = 2.5
vehicle.airspeed=2.5

arm_and_takeoff(vehicle,10)

# Mission Execution:
missionfile = "day2.waypoints"
m_lst = upload_mission(vehicle,missionfile) # generating mission list
#mission_exe(vehicle,m_lst)
max_cmd_id = len(m_lst)-1

# Run mission in a separate thread
event = threading.Event()
event.clear()
thread1 = threading.Thread(target=mission_exe,args=(vehicle,m_lst,event),daemon=True) # Mission runs in auto mode
thread1.start()

# names: ['hotspot', 'target']
# Video Feed and YOLO
model = YOLO("./yolo_weights/old_fine.pt") # Load Model
vid = cv2.VideoCapture(0)

# video feed configuration:
vid.set(cv2.CAP_PROP_FRAME_WIDTH,640)   # mid = 320
vid.set(cv2.CAP_PROP_FRAME_HEIGHT,480)   # mid = 240

while True:
    _,frame = vid.read()
    
    cv2.circle(frame,(320,240),2,(0,255,0),3)

    # Required variables:
    (cx,cy) = (None,None)

    # Inference
    for result in model.predict(source=frame,stream=True,device=0,show=True):
        # if prev_found:
        #     break

        org_frame = result.orig_img
        for box in result.boxes:
            b = box.xyxy[0]
            conf=round(float(box.conf),2)
            cls=box.cls
            #print(cls)
            if conf >= 0.75:
                if model.names[int(cls)] == "hotspot":
                    if count_h == 0:
                        hotspot_flag = True
                    else:
                        end = time.time()
                        diff = end - start
                        if diff < 15:
                            hotspot_flag = False
                        else:
                            hotspot_flag = True

                    x1,y1,x2,y2=b
                    (cx,cy) = (int((x1+x2)/2),int((y1+y2)/2))  # (cx,cy) is the centre point of object
                    cv2.circle(frame,(cx,cy),2,(0,255,0),3)
                    break
                if model.names[int(cls)] == "target" and target_not_found:
                    target_flag = True
                    x1,y1,x2,y2=b
                    (cx,cy) = (int((x1+x2)/2),int((y1+y2)/2))  # (cx,cy) is the centre point of object
                    cv2.circle(frame,(cx,cy),2,(0,255,0),3)
                    break

    if hotspot_flag:
        vehicle.mode = VehicleMode("GUIDED")
        try:
            (cx,cy) = (int(cx),int(cy))
        except TypeError:
            continue
        diff = pixel_diff(cx,cy)
        # function to auto adjust -> drone with hotspot
        if (diff > 10):
            if (cx,cy) < (320,240):  # center in plane 1
                plane1(vehicle)  
            elif (cx > 320) and (cy < 240): # center in plane 2
                plane2(vehicle)
            elif (cx < 320) and (cy > 240): # center in plane 3
                plane3(vehicle)
            elif (cx,cy) > (320,240): # center in plane 4
                plane4(vehicle)
            loop=True
            continue


        # Decent to 10m
        decent_10(vehicle)

        # Save the image
        cv2.imwrite(f"./hotspot_image/hotspot_image{i}.jpg",frame)
        i+=1
        count_h = 1

        # Increment to 30m
        increase_alt_30(vehicle)
        hotspot_flag = False

        # Start timer
        start = time.time()

        vehicle.mode = VehicleMode("AUTO")
        #prev_found = True


    if target_flag and target_not_found:
        vehicle.mode = VehicleMode("GUIDED")
        try:
            (cx,cy) = (int(cx),int(cy))
        except TypeError:
            continue
        diff = pixel_diff(cx,cy)
        # function to auto adjust -> drone with target
        if (diff > 10):
            if (cx,cy) < (320,240):  # center in plane 1
                plane1(vehicle)  
            elif (cx > 320) and (cy < 240): # center in plane 2
                plane2(vehicle)
            elif (cx < 320) and (cy > 240): # center in plane 3
                plane3(vehicle)
            elif (cx,cy) > (320,240): # center in plane 4
                plane4(vehicle)
            loop=True
            continue


        # Decent to 20m
        decent_20(vehicle)

        # drop the payload -> servo
        controlServo(vehicle,9,1100)
        time.sleep(3)

        # Increment to 30m
        increase_alt_30(vehicle)
        target_flag = False
        target_not_found = False  # setting that the target has already found 
        #vehicle.mode = VehicleMode("AUTO")

    cv2.imshow("OUT",frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        print("Exiting Vid")
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(3)
        break
    
    # if waypoint_reached():
    #     prev_found = False

    event.set()
    #time.sleep(10)


vid.release()


#print("Pass for 10 seconds")
#time.sleep(10)


# RTL
return_to_launch(vehicle)

# Closing Vehicle object
vehicle.close()
