#!/usr/bin/env python
import rospy
import math
import autobot
import operator
from autobot.msg import drive_param
from sensor_msgs.msg import LaserScan
from autobot.msg import pid_input
from autobot.msg import wall_dist
from autobot.msg import pathFinderState
from autobot.srv import *

"""
TODO:
- [x] Decide if you want to hug right/left/or closest wall
    - Right wall hugged for now to simulate right side of road
    - Update: wall decision to be made by image processing node
- [x] Send error left to hug left wall
- [ ] Use a command line argument to enable/disable debug msgs
"""


class PathConfig(object):
    __slots__ = ('wallToWatch', 'desiredTrajectory', 'velocity', 'pubRate',
                 'minFrontDist', 'enabled')
    """
    wallToWatch: Set which wall to hug
    options: autobot.msg.wall_dist.WALL_LEFT
             autobot.msg.wall_dist.WALL_RIGHT
             autobot.msg.wall_dist.WALL_FRONT  #< probably won't be used
    """
    def __init__(self):
        self.wallToWatch = autobot.msg.wall_dist.WALL_RIGHT
        self.desiredTrajectory = 0.5  # desired distance from the wall
        self.minFrontDist = 1.4       # minimum required distance in front of car
        self.velocity = 10070          # velocity of drive
        self.pubRate = 0              # publish rate of node
        self.enabled = True          # enable/disable state of wall hugging

PATH_CONFIG = PathConfig()
errorPub = rospy.Publisher('error', pid_input, queue_size=10)
motorPub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
statePub = rospy.Publisher('pathFinderStatus', pathFinderState, queue_size=10)


def HandleTogglePathFinderService(req):
    """ Handler for enabling/disabling path finder
    Responds with ack msg (bool)
    """
    global PATH_CONFIG
    PATH_CONFIG.enabled = True   #req.state
    return TogglePathFinderResponse(True)


def HandleAdjustWallDist(req):
    """ Handler for adjusting wall hugging parameters

    Responds with wall_dist msg and a bool to verify that the
    service command has been accepted
    """
    global PATH_CONFIG

    # print " wall {}".format(req.cmd.wall)
    # print " dist {}\n".format(req.cmd.dist)

    resp = wall_dist()
    isValid = req.cmd.dist >= 0

    if isValid is True and req.cmd.wall != autobot.msg.wall_dist.WALL_FRONT:
        """ only accept WALL_LEFT or WALL_RIGHT
        Service client can send an invalid wall or distance
        query current settings
        """
        if req.cmd.wall is not wall_dist.WALL_UNDEF:
            PATH_CONFIG.wallToWatch = req.cmd.wall

        PATH_CONFIG.desiredTrajectory = req.cmd.dist
    else:
        isValid = False

    resp.wall = PATH_CONFIG.wallToWatch
    resp.dist = PATH_CONFIG.desiredTrajectory
    return AdjustWallDistResponse(resp, isValid)


def publishCurrentState(event):
    global PATH_CONFIG

    msg = pathFinderState()
    msg.velocity = PATH_CONFIG.velocity
    msg.hug.wall = PATH_CONFIG.wallToWatch
    msg.hug.dist = PATH_CONFIG.desiredTrajectory
    msg.enabled = True   #PATH_CONFIG.enabled
    statePub.publish(msg)


def getRange(data, theta):
    """ Find the index of the array that corresponds to angle theta.
    Return the lidar scan value at that index
    Do some error checking for NaN and absurd values
    data: the LidarScan data
    theta: the angle to return the distance for
    """
    car_theta = math.radians(theta) - math.pi / 2
    if car_theta > 3 * math.pi / 4:
        car_theta = 3 * math.pi / 4
    elif car_theta < -3 * math.pi / 4:
        car_theta = -3 * math.pi / 4

    float_index = (car_theta + 3 * math.pi / 4) / data.angle_increment
    index = int(float_index)
    return data.ranges[index]

def max_list(list):
    maxelem = []
    highest = 0
    for x in list:
        if highest < x[2]:
            highest = x[2]
            maxelem = x
    return maxelem

# Function will return the biggest free available segment
def far_see(data):
    rangeDist = 5       #checking distance range is 5
    i = 0
    #count = 0
    dic = []
    start_seg = 0
    for count, x in enumerate(data):
        if x > rangeDist:
            if i is 0:
                start_seg = count
            i += 1
        else:
            #dic.append([start_seg, count, i])
            if i is not 0:
                dic.append([start_seg, count, i])
                i = 0
     
    #key_max = max(dic.keys(), key=(lamda k: dic[k]))
    #key_max = max(dic.items(), key=operator.itemgetter(1))[0]
    #key_min = min(dic.keys(), key=(lamda k: dic[k]))
    key_max = max_list(dic)

    if len(key_max) is 3:
        computed_seg_first = key_max[0]
        computed_seg_last = key_max[1] 
        print("key_len = 3")
        if key_max[2] > 6:         
            mid_seg = (computed_seg_first+computed_seg_last)/2
        else:
            mid_seg = 0
    else:
        mid_seg = 0

    return mid_seg



def short_see(direc, data):

    i_rl = 0

    while direc < 120 and direc > 60:
        near_vision = [getRange(data, direc-30+i) for i in range(60)]
        if any(value < 0.5 for value in near_vision):
            i_rl += 1
            if i_rl % 2:
                direc = direc+i_rl
            else:
                direc = direc-i_rl
        else:
            return direc

    return -1     


def callback(data):
    global PATH_CONFIG

    # Do not attempt to hug wall if disabled
    if PATH_CONFIG.enabled is False:
        return

    #frontDistance = getRange(data, 90)
    car_vision = [getRange(data, 60+i) for i in range(60)]

    direc = far_see(car_vision)

    driveParam = drive_param()

    if direc is 0:
        driveParam.velocity = 9830
    else:
        
        d = short_see(direc+60, data)
        print("short_ direc = ", d)
        
        if d is -1:
            driveParam.velocity = 9830 #sudden stop and include rev flag
            driveParam.angle = 15
        else: 
            driveParam.velocity = PATH_CONFIG.velocity
            driveParam.angle = 3*(90-d)
            #if d < 90:
             #   driveParam.angle = 3*(90-d)
            #elif d > 90:
               # driveParam.angle = 3*(90-d)

    print("velocity = ", driveParam.velocity)
    print("angle = ", driveParam.angle)
    print("far direc = ", direc)
     

    motorPub.publish(driveParam)
    return


if __name__ == '__main__':
    print("Path finding node started")
    rospy.Service('adjustWallDist', AdjustWallDist, HandleAdjustWallDist)
    rospy.Service('togglePathFinder', TogglePathFinder,
                  HandleTogglePathFinderService)
    rospy.init_node('pathFinder', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.Timer(rospy.Duration(0.5), callback=publishCurrentState)
    rospy.spin()
