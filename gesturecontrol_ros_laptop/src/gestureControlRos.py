#!/usr/bin/env python3

import cv2
import cvzone
import numpy as np
from cvzone.ColorModule import ColorFinder
import rospy
from geometry_msgs.msg import Twist

height = 720
width = 1280

rospy.init_node('gesture_commands_publish')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
rate = rospy.Rate(100)
cap = cv2.VideoCapture(0)

cap.set(3, width)
cap.set(4, height)

topClearance = 100
sideClearance = 40

dirLimits = (sideClearance, int((width / 2) - sideClearance))
movLimits = (topClearance, height - topClearance)

myColourFinder = ColorFinder(False)

redHsvVals = {'hmin': 138, 'smin': 68, 'vmin': 96, 'hmax': 179, 'smax': 156, 'vmax': 255}
blueHsvVals = {'hmin': 108, 'smin': 120, 'vmin': 104, 'hmax': 123, 'smax': 237, 'vmax': 255}
greenHsvVals = {'hmin': 64, 'smin': 73, 'vmin': 81, 'hmax': 89, 'smax': 241, 'vmax': 177}

dirVal = 0
movVal = 0

commands = {'mov':0, 'dir':0}


def draw_basic(frame):
    frame = cv2.line(frame, (int(width / 2), 0), (int(width / 2), height), (0, 0, 0), thickness=3)

    frame = cv2.line(frame, (sideClearance, (height - 60)), (int((width / 2) - sideClearance), (height - 60)),
                     (255, 0, 0), thickness=2)
    frame = cv2.line(frame, (sideClearance, (height - 68)), (sideClearance, (height - 52)), (0, 0, 255), thickness=4)
    frame = cv2.line(frame, (int((width / 2) - sideClearance), (height - 68)),
                     (int((width / 2) - sideClearance), (height - 52)), (0, 0, 255), thickness=4)
    frame = cv2.line(frame, (int(width / 4), (height - 63)), (int(width / 4), (height - 57)), (0, 255, 0), thickness=3)

    frame = cv2.line(frame, (int((width / 2) + 40), topClearance), (int((width / 2) + 40), (height - topClearance)),
                     (255, 0, 0), thickness=2)
    frame = cv2.line(frame, (int((width / 2) + 32), topClearance), (int((width / 2) + 48), topClearance), (0, 0, 255),
                     thickness=4)
    frame = cv2.line(frame, (int((width / 2) + 32), (height - topClearance)),
                     (int((width / 2) + 48), (height - topClearance)), (0, 0, 255), thickness=4)
    frame = cv2.line(frame, (int((width / 2) + 37), int(height / 2)), (int((width / 2) + 43), int(height / 2)),
                     (0, 255, 0), thickness=4)
    return frame

def getData(img, hsvVals):
    imgColor, mask = myColourFinder.update(img, hsvVals)
    imgContour, contours = cvzone.findContours(img, mask)

    if contours:
        bBox = contours[0]['bbox']
        return bBox

def getCenter(bBox):
    return int(bBox[0] + (bBox[2] / 2)), int(bBox[1] + (bBox[3] / 2))


while True: 
    ret, img = cap.read()
    img = cv2.flip(img, 1)
    img = draw_basic(img)
    twist = Twist()

    movBbox = getData(img, blueHsvVals)
    if movBbox:
        cx, cy = getCenter(movBbox)
        img = cv2.rectangle(img, (movBbox[0], movBbox[1]), (movBbox[0] + movBbox[2], movBbox[1] + movBbox[3]),
                            (255, 0, 0), thickness=2)
        img = cv2.circle(img, (cx, cy), 4, (255, 0, 0), thickness=-1)
        if movLimits[0] <= cy <= movLimits[1] and cx >= int(width / 2):
            movVal = round(np.interp(cy, [movLimits[0], movLimits[1]], [100, -100]), 1)
            img = cv2.putText(img, str(movVal) + ' %', (int((width/2)+50), int(height/2)+7), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 90, 255), 3)
            img = cv2.line(img, (int((width / 2) + 40), int(height / 2)), (int((width / 2) + 40), cy), (90, 255, 120), thickness=4)
            commands['mov'] = movVal
        else:
            commands['mov'] = 0
    else:
        commands['mov'] = 0

    dirBbox = getData(img, greenHsvVals)
    if dirBbox:
        cx, cy = getCenter(dirBbox)
        img = cv2.rectangle(img, (dirBbox[0], dirBbox[1]), (dirBbox[0] + dirBbox[2], dirBbox[1] + dirBbox[3]),
                            (0, 255, 0), thickness=2)
        img = cv2.circle(img, (cx, cy), 4, (0, 255, 0), thickness=-1)
        if dirLimits[0] <= cx <= dirLimits[1] and cx <= int(width / 2):
            dirVal = round(np.interp(cx, [dirLimits[0], dirLimits[1]], [-100, 100]), 1)
            img = cv2.putText(img, str(dirVal) + ' %', (int(width/4)-30, int(height-70)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 90, 255), 3)
            img = cv2.line(img, (int(width / 4), int(height-60)), (cx, int(height-60)), (90, 255, 120), thickness=4)
            commands['dir'] = dirVal
        else:
            commands['dir'] = 0
    else:
        commands['dir'] = 0
    

    twist.linear.x = commands['mov']
    twist.angular.z = commands['dir']

    pub.publish(twist)
    rate.sleep()

    cv2.imshow("Image", img)
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break
