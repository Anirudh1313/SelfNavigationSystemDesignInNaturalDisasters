import roslib #roslib.load_manifest('numpy_tutorials') #not sure why I need this
import rospy
import autobot
from autobot.msg import gps_direc
from std_msgs.msg import String
import serial
import math
from math import cos, sin, atan2, sqrt


ser = serial.Serial('/dev/ttyACM1', 9600)

RAD = 3.14159265/180

def bearing_angle(lat1, long1, lat2, long2):
    long_diff = long2 * 3.14159265 / 180 - long1 * 3.14159265 / 180
    x = math.cos(lat2 * 3.14159265 / 180) * sin(long_diff)
    y = math.cos(lat1 * 3.14159265 / 180) * sin(lat2 * 3.14159265 / 180) - sin(lat1 * 3.14159265 / 180) * cos(lat2 * 3.14159265 / 180) * cos(long_diff)
    #x = sin(90 * 3.14159265 / 180);
    bearing = math.atan2(x, y)
    return bearing * 180 / 3.14159265

def distance(lat1, long1, lat2, long2):
    radius = 6371000;   #in metres
    #converting degrees to radians
    #calculating the diff of latitudes and longitudes
    long_diff = long2*3.14159265/180 - long1*3.14159265/180
    lat_diff = lat2*3.14159265/180 - lat1*3.14159265/180
    lat1 = lat1*3.14159265/180
    long1 = long1*3.14159265/180
    lat2 = lat2*3.14159265/180
    long2 = long2*3.14159265/180
    a = math.sin(lat_diff/2)*math.sin(lat_diff/2) + math.cos(lat1)*math.cos(lat2)*math.sin(long_diff/2)*math.sin(long_diff/2)
    c = 2*math.atan2(sqrt(a), math.sqrt(1-a))
    distance = radius*c91
    return distance    

def talker():
    heading = 0
    lat_1 = 0
    long_1 = 0
    while not rospy.is_shutdown():
        data= ser.readline() # I have "hi" coming from the arduino as a test run over the serial port
        rospy.loginfo(data)
        print(data)
        if (data.startswith("Lat:")):
            lat_1 = float(data.split(":")[1])
        elif (data.startswith("Long:")):
            long_1 = float(data.split(":")[1])
        elif (data.startswith("angle:")):
            heading = float(data.split(":")[1])
        else:
            lat_1 = 0
            long_1 = 0
        
        lat_2 = 37.335761
        long_2 = -121.881639
        if (lat_1 != 0 and long_1 != 0):
            angle = bearing_angle(lat_1, long_1, lat_2, long_2)
            distance = distance(lat_1, long_1, lat_2, long_2)   #in meters

            print(angle)
            print(distance)

            degree = angle - heading

            dir_msg.angle = 0
            dir_msg.velocity = 10070
            if ((0 < degree and degree <= 5) or (-360 <= degree and degree < -355)):
                dir_msg.angle = 0 # go straight 3
            elif ((5 < degree and degree <= 15) or (-355 <= degree and degree < -345)):
                dir_msg.angle = -10 # slight left: 5 degrees 2
            elif ((15 < degree and degree <= 60) or (-345 <= degree and degree < -300)):
                dir_msg.angle = -30 # mid left: 15 degrees 1
            elif ((60 < degree and degree <= 180) or (-300 <= degree and degree < -180)):
                dir_msg.angle = -90 # full left: 60 degrees 0
            elif ((355 < degree and degree <= 360) or (-5 <= degree and degree < 0)):
                dir_msg.angle = 0 # go straight 3
            elif ((345 < degree and degree <= 355) or (-15 <= degree and degree < -5)):
                dir_msg.angle = 10 # slight right: 5 degrees 4
            elif ((300 < degree and degree <= 345) or (-60 <= degree and degree < -15)):
                dir_msg.angle = 30 # midright: 15degrees 5
            elif ((180 < degree and degree <= 300) or (-180 <= degree and degree <= -60)):
                dir_msg.angle = 90 #full right: 60 degrees 6
            elif (distance < 10): # Stop if destination is within 10 mtrs
                dir_msg.angle = 0 # stop the motors, not able to calaculate correct angle to turn
                dir_msg.velocity = 9830
            else:
                dir_msg.angle = 0 # stop the motors, not able to calaculate correct angle to turn
                dir_msg.velocity = 9830
            
            pub.publish(dir_msg)
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
      pub = rospy.Publisher('GPS', String, queue_size=10)
      rospy.init_node('gps')
      talker()
    except rospy.ROSInterruptException:
      pass

