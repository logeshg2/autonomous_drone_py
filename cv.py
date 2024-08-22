import cv2
import time
from movement import forward,backward,right,left
from ultralytics import YOLO


# def cv_fun():
#     vid = cv2.VideoCapture(0)
#     while True:
#         _,frame = vid.read()
#         cv2.imshow("OUT",frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             cv2.destroyAllWindows()
#             print("Hello")
#             break
#     vid.release()

def pixel_diff(x2,y2):
    (x,y)=(320,240)
    dist = ((x-x2)**2 + (y-y2)**2 )**0.5
    return dist

# Plane 1
# (320,0) & (0,240)
def plane1(vehicle):
    forward(vehicle,5)
    left(vehicle,5)

# Plane 2
# (320,0) & (640,240)
def plane2(vehicle):
    forward(vehicle,5)
    right(vehicle,5)

# Plane 3
# (0,240) & (320,480)
def plane3(vehicle):
    backward(vehicle,5)
    left(vehicle,5)

# Plane 4
# (320,480) & (640,240)
def plane4(vehicle):
    backward(vehicle,5)
    right(vehicle,5)