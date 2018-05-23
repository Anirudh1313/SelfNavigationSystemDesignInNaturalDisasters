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
from autobot.msg import gps_direc

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
                 'minFrontDist', 'enabled', 'rev', 'count', 'switch_to_gps')
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
        self.rev = False
        self.count = 0
        self.switch_to_gps = False

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
    i_rl = 0
    direc = 90
    # checking for broad angle, from starting from center, if front is completely blocked then the segment starts turning sideways
    while direc < 120 and direc > 60:
        near_vision = [getRange(data, direc-3+i) for i in range(6)]
        if any(value < 5 for value in near_vision):
            i_rl += 1
            if i_rl % 2:
                direc = direc+i_rl
            else:
                direc = direc-i_rl
        else:
            return direc

    return -1 



def short_see(direc, data):

    i_rl = 0

    while direc < 120 and direc > 60:
        near_vision = [getRange(data, direc-15+i) for i in range(30)]
        if any(value < 0.9 for value in near_vision):
            i_rl += 1
            if i_rl % 2:
                direc = direc+i_rl
            else:
                direc = direc-i_rl
        else:
            return direc

    return -1     

def check_2m_and_run(data):

    global PATH_CONFIG
    farFrontDist = [getRange(data, 80+i) for i in range(20)]			# creates the list of all the FAR-FRONT values in range 80-to-100
    frontDistance = [getRange(data, 70+i) for i in range(40)]			# creates the list of all the FRONT 	 values in range 70-to-110

    #theta = 50  # PICK THIS ANGLE TO BE BETWEEN 0 AND 70 DEGREES

    #thetaDistRight = getRange(data, theta)  # a
    rightDist = [getRange(data, 50+i) for i in range(30)]  # b

    #thetaDistLeft = getRange(data, 180-theta)  # aL
    leftDist = [getRange(data, 100+i) for i in range(30)]  # bL

    left_b = False																		# INDICATES IF LEFT  is blocked
    right_b = False																		# INDICATES IF RIGHT is blocked

    minLeftDist = 1				# threshold for LEFt distance
    minRightDist = 1				# threshold for RIGHT  distance
    minFrontDist = 0.5			# threshold for FRONT distance
    minFarFrontDist = 2			# threshold for FarFront distance

    #driveParam = drive_param()

 
    #looking for closer objest, go to blocked state

    velocity = PATH_CONFIG.velocity
    angle = 11


    if any(front_fd < minFarFrontDist for front_fd in farFrontDist):		# CHECKs IF FAR-front is blocked
        print "Far Front Blocked!"
        
#        driveParam.velocity = PATH_CONFIG.velocity
#        smallest = 100;
#        for d in rightDist:
#            if smallest > d:
#                smallest = d

        fd = min(farFrontDist)		#returns the MINIMUM distance from the list containing all the FAR-FRONT DISTANCE values
        if any(front_d < minFrontDist for front_d in frontDistance):		# CHECKs IF Front is blocked
            print "Front Blocked!"
            if not PATH_CONFIG.rev:
                velocity = 6600
                PATH_CONFIG.count += 1
                if PATH_CONFIG.count > 10:
                    PATH_CONFIG.rev = True
                    PATH_CONFIG.count = 0
            else:
                velocity = 9830

            angle = 0
        elif all(left_d > min(rightDist) for left_d in leftDist):
            angle = -90 + (fd-1)*60
            print "Turning left"
            PATH_CONFIG.rev = False
            PATH_CONFIG.count = 0
        elif all(right_d > min(leftDist) for right_d in rightDist):
            angle = 90 - (fd-1)*60
            print "Turning right"
            PATH_CONFIG.rev = False
            PATH_CONFIG.count = 0
        else:
            velocity = 10000 
            angle = 0
            PATH_CONFIG.rev = False
            PATH_CONFIG.count = 0
    else:
        PATH_CONFIG.rev = False
        PATH_CONFIG.count = 0
        
    
    vel_ang = []
    vel_ang.append(velocity)
    vel_ang.append(angle)

    return vel_ang


def callback(data):
    global PATH_CONFIG

    # Do not attempt to hug wall if disabled
    if PATH_CONFIG.enabled is False:
        return

    #frontDistance = getRange(data, 90)

    #checking for far front distance
    farFrontDist = [getRange(data, 85+i) for i in range(10)]	

    # if frontDist is less than 5m, then go in
    driveParam = drive_param()
    direc = 90
    fd = min(farFrontDist)

    #initially setting default velocity and angle, change in appropriate conditions
    driveParam.velocity = PATH_CONFIG.velocity
    driveParam.angle = 0

    if fd < 5:
        PATH_CONFIG.switch_to_gps = False
        if fd < 2:
            vel_ang = check_2m_and_run(data)

            driveParam.velocity = vel_ang[0]
            driveParam.angle = vel_ang[1] 
        else:
             #check for available largest segment and get the direction
            direc = far_see(data)

        
            if direc is -1:  #checking for return value from far_see
                #driveParam.velocity = 9830
                vel_ang = check_2m_and_run(data)

                driveParam.velocity = vel_ang[0]
                driveParam.angle = vel_ang[1]          
            else:
                #turn the car to the free space direction, then automatically farFrontDist goes higher than 5, will go out of this main if loop
    
                # LiDAR direc is from 0 to 180 in front semicircle, for angle pwm its from 90 to -90
                if direc < 90:   #if far_see output direction is towards right side
                    # turning car slowly from, 30' to 90' by checking the min front dist from 5m to 2m. so Instantly it turns to 30 and then turns to slowly to 90
                    # direc is in LiDAR angle bangwidth, converting it to pwm angle bandwidth as well
                    driveParam.angle = (90 - direc) - (fd-2)*11.25
                    PATH_CONFIG.rev = False
                    PATH_CONFIG.count = 0
                else:
                    # turning car slowly from, -30' to -90' by checking the min front dist from 5m to 2m.
                    driveParam.angle = (90 - direc) + (fd-2)*11.25
                    PATH_CONFIG.rev = False
                    PATH_CONFIG.count = 0
 
        motorPub.publish(driveParam)
    else:
        frontDistance = [getRange(data, 70+i) for i in range(40)]			# creates the list of all the FRONT 	 values in range 70-to-110
        if any(front_d < 2 for front_d in frontDistance):
            vel_ang = check_2m_and_run(data)
            PATH_CONFIG.switch_to_gps = False
            driveParam.velocity = vel_ang[0]
            driveParam.angle = vel_ang[1] 
            motorPub.publish(driveParam)
        else:
            PATH_CONFIG.switch_to_gps = True

    print("velocity = ", driveParam.velocity)
    print("angle = ", driveParam.angle)
    print("far direc = ", direc)
   
    return

def gps_callback(gps_direc):
    global PATH_CONFIG
    if PATH_CONFIG.switch_to_gps:
        driveParam.velocity = gps_direc.velocity
        driveParam.angle = gps_direc.angle 
        motorPub.publish(driveParam)

if __name__ == '__main__':
    print("Path finding node started")
    rospy.Service('adjustWallDist', AdjustWallDist, HandleAdjustWallDist)
    rospy.Service('togglePathFinder', TogglePathFinder, HandleTogglePathFinderService)
    rospy.init_node('pathFinder', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.Subscriber("GPS", gps_direc, gps_callback)
    rospy.Timer(rospy.Duration(0.5), callback=publishCurrentState)
    rospy.spin()
