#!/usr/bin/env python

import rospy
from autobot.msg import drive_param
from autobot.msg import wall_dist
from autobot.msg import pathFinderState
from autobot.msg import detected_object
from autobot.srv import *
from sensor_msgs.msg import Image
from pathFinder import PathConfig
from obstruction import *
from stopsign import *

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

"""
This node is responsible for configuring the pathFinder node
when an object is detected.
"""

PATH_STATE = PathConfig()
PUB_DRIVE = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
OBJECT_MAP = ObstructionMap()
STOP_LOGIC = StopSign()
DEFAULT_WALL_DIST = 0.5


def togglePathFinder(state):
    try:
        rospy.wait_for_service('togglePathFinder', timeout=0.2)
        srv = rospy.ServiceProxy('togglePathFinder', TogglePathFinder)
        srv(state)  # ignore ACK response
    except rospy.ROSException, e:
        # print "Service called failed: %s" % e
        pass


def stopCar():
    global PUB_DRIVE
    togglePathFinder(False)
    msg = drive_param()
    msg.velocity = 0
    msg.angle = 0
    PUB_DRIVE.publish(msg)


def setWallDist(dist, wall):
    try:
        rospy.wait_for_service('adjustWallDist')
        adjustWall = rospy.ServiceProxy('adjustWallDist', AdjustWallDist)
        cmd = wall_dist()
        cmd.wall = wall
        cmd.dist = dist
        resp = adjustWall(cmd)
        return resp
    except rospy.ROSException, e:
        print "Service called failed: %s" % e
        pass


def convertWallToString(wall):
    # WALL_LEFT=0
    # WALL_FRONT=1
    # WALL_RIGHT=2
    if (wall is wall_dist.WALL_LEFT):
        return "Left"
    elif (wall is wall_dist.WALL_RIGHT):
        return "Right"
    elif (wall is wall_dist.WALL_FRONT):
        return "Front"
    else:
        return "Unknown"


def onPathFinderUpdated(status):
    global PATH_STATE
    PATH_STATE.velocity = status.velocity
    PATH_STATE.wallToWatch = status.hug.wall
    PATH_STATE.desiredTrajectory = status.hug.dist
    PATH_STATE.enabled = status.enabled


def getAverageColor(img):
    """Returns average color of img"""
    avgColorPerRow = np.average(img, axis=0)
    avgColor = np.average(avgColorPerRow, axis=0)
    return avgColor


def shadeToDepth(color):
    """Returns depth in meters from color b/w"""
    minDistance = 0.7
    maxDistance = 20
    maxColor = 255
    color = np.average(color, axis=0)
    # depth = mx + b
    m = (minDistance - maxDistance)/maxColor
    x = color
    b = maxDistance
    return m * x + b


def hasObstruction(className, list):
    for o in list:
        if o.className == className:
            return (True, o)

    return (False, None)


def onDecisionInterval(event):
    """Makes pathing decision based on objects detected"""
    global OBJECT_MAP
    global PATH_STATE
    global STOP_LOGIC

    dangers = OBJECT_MAP.getHighPriorities()
    if STOP_LOGIC.state == StopStates.FULL_STOP:
        return

    hasDog, obstruction = hasObstruction('dog', dangers)
    # TODO: make sure person is in a certain X position before stopping
    if hasDog and obstruction.distance < 8:
        stopCar()
        OBJECT_MAP.clearMap()
        return  # a person has priority over all

    hasStop, stopSign = hasObstruction('stop sign', dangers)
    if (hasStop and stopSign.distance < 8 and
            STOP_LOGIC.state != StopStates.IGNORE_STOP_SIGNS):
        if STOP_LOGIC.state == StopStates.NORMAL:
            stopCar()
            STOP_LOGIC.stopSignDetected()

        OBJECT_MAP.clearMap()
        return

    wallHug = PATH_STATE.wallToWatch
    sideToCheck = (ObstructionMap.RIGHT if
                   PATH_STATE.wallToWatch == wall_dist.WALL_RIGHT
                   else ObstructionMap.LEFT)

    closest = OBJECT_MAP.getClosestOnSide(sideToCheck)
    if closest is not None and closest.className == 'door':
        setWallDist(4.0, wall_dist.WALL_UNDEF)
        OBJECT_MAP.clearMap()
        return

    # Fallback to normal wall route mode
    # Do you want to revert to the default distance here?
    # global DEFAULT_WALL_DIST
    # setWallDist(DEFAULT_WALL_DIST, wall_dist.WALL_UNDEF)
    setWallDist(PATH_STATE.desiredTrajectory, wall_dist.WALL_UNDEF)
    togglePathFinder(True)
    pathStateUpdated = False

    OBJECT_MAP.clearMap()


def onObjectDetected(msg):
    """
    message type == detected_object.msg

    m.class: str
    m.depthImg: image
    m.box: bounding_box
    """
    bridge = CvBridge()
    try:
        depthMap = bridge.imgmsg_to_cv2(msg.depthImg,
                                        desired_encoding="passthrough")
        # Get the center crop of the boxed image
        startY = int(msg.box.height//4)
        startX = int(msg.box.width//4)
        crop = depthMap[startY: startY + int(msg.box.height//2),
                        startX: startX + int(msg.box.width//2)]
        avg = getAverageColor(crop)
        distance = shadeToDepth(avg)
        global OBJECT_MAP
        OBJECT_MAP.addToMap(msg.className,
                            msg.box.origin_x, msg.box.origin_y,
                            distance)
    except CvBridgeError as e:
        print(e)


if __name__ == '__main__':
    DECISION_RATE_SEC = 0.70
    rospy.init_node('navigator', anonymous=True)
    rospy.Subscriber("pathFinderStatus", pathFinderState, onPathFinderUpdated)
    # rospy.Subscriber("drive_parameters", drive_param, driveParamsUpdated)
    rospy.Subscriber("object_detector", detected_object, onObjectDetected)
    rospy.Timer(rospy.Duration(DECISION_RATE_SEC), callback=onDecisionInterval)
    rospy.spin()

