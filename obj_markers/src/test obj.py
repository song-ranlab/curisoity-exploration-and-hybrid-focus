#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from darknet_ros_msgs.msg import BoundingBoxes
from nav_msgs.msg import OccupancyGrid, Odometry
import tf


#May or may not need this import
import roslib; roslib.load_manifest('visualization_marker_tutorials')

#Global Variables:
topic = 'obj_interest_marker'
hdg = 90 #HDG set to -1 as place holder until desired object detected
yaw = 0

xpos = 0
ypos = 0
robx = 0 
roby = 0

marker = Marker()
id = 0
pub = rospy.Publisher(topic, Marker, queue_size=10)
m = 0

#Testing variables
xcen = 50
sec = 0
hdg = int(10)
ort = "R"

def Lcallback(msg):
    
    #define global variables to pass information in and out
    global xpos, ypos
    global hdg, yaw
    global sec, check, ort
    
    print("HDG = ", hdg)
    #pull range data from LIDAR
    ran = msg.ranges[350]
    print("ran =", ran)
    #Determine quadrent that robot is pointing
    if yaw < 0:
        yaw = abs(yaw)
        if yaw <= (math.pi/2):
            sec = 1
        else:
            sec = 2
    elif yaw > 0:
        yaw = abs(yaw)
        if yaw <= (math.pi/2):
            sec = 3
        else:
            sec = 4

    print("SECTOR ", sec)

    print("YAW = ", yaw*(180/math.pi))

    if (sec == 1 and ort == "R"):
        hd = hdg*(math.pi/180)
        comp = yaw*(180/math.pi) + hdg
        if comp > 90:
            alpha = abs((yaw + hd) - (math.pi/2))
            xpos = (-1)*ran*math.sin(alpha) + robx
            ypos = (-1)*ran*math.cos(alpha) + roby
        else:
            alpha = abs(yaw + hd)
            xpos = ran*math.cos(alpha) + robx
            ypos = (-1)*ran*math.sin(alpha) + roby
    elif (sec == 3 and ort == "L"):
        hd = hdg*(math.pi/180)
        comp = ((yaw*(180/math.pi) + hdg))
        if comp > 90:
            alpha = abs(yaw + hd - (math.pi/2))
            xpos = (-1)*ran*math.sin(alpha) + robx
            ypos = ran*math.cos(alpha) + roby
        else:
            alpha = abs(yaw + hd)
            xpos = ran*math.cos(alpha) + robx 
            ypos = ran*math.sin(alpha) + roby
    elif (sec == 1 and ort == "L"):
        hd = hdg*(math.pi/180)
        comp = ((yaw*(180/math.pi))- hdg)
        if comp < 0:
            alpha = abs(yaw - hd)
            xpos = (-1)*ran*math.sin(alpha) + robx
            ypos = ran*math.cos(alpha) + roby
        else:
            alpha = abs(yaw - hdg)
            xpos = ran*math.cos(alpha) + robx
            ypos = (-1)*ran*math.sin(alpha) + roby
    elif (sec == 3 and ort == "R"):
        hd = hdg*(math.pi/180)
        comp = ((yaw*(180/math.pi))- hdg)
        if comp < 0:
            alpha = abs(yaw - hd)
            xpos = ran*math.cos(alpha) + robx
            ypos = (-1)*ran*math.sin(alpha) + roby
        else:
            alpha = abs(yaw - hd)
            xpos = ran*math.cos(alpha) + robx
            ypos = ran*math.sin(alpha) + roby
    elif (sec == 2 and ort == "R"):
        hd = hdg*(math.pi/180)
        comp = ((yaw*(180/math.pi)) + hdg)
        if comp > 180:
            alpha = abs(math.pi - yaw - hd)
            xpos = (-1)*ran*math.cos(alpha) + robx
            ypos = ran*math.sin(alpha) + roby
        else:
            alpha = abs(math.pi - yaw - hd)
            xpos = (-1)*ran*math.cos(alpha) + robx
            ypos = (-1)*ran*math.sin(alpha) + roby
    elif (sec == 4 and ort == "L"):
        hd = hdg*(math.pi/180)
        comp = ((yaw*(180/math.pi)) + hdg)
        if comp > 180:
            alpha = abs(math.pi - yaw - hd)
            xpos = (-1)*ran*math.cos(alpha) + robx
            ypos = (-1)*ran*math.sin(alpha) + roby
        else:
            alpha = abs(math.pi - yaw - hd)
            xpos = (-1)*ran*math.cos(alpha) + robx
            ypos = ran*math.sin(alpha) + roby
    elif (sec == 2 and ort == "L"):
        hd = hdg*(math.pi/180)
        comp = ((yaw*(180/math.pi)) - hdg)
        if comp < 90:
            alpha = abs(yaw - hd)
            xpos = ran*math.cos(alpha) + robx
            ypos = (-1)*ran*math.sin(alpha) + roby
        else:
            alpha = abs(yaw - (math.pi/2) - hd)
            xpos = (-1)*ran*math.sin(alpha) + robx
            ypos = (-1)*ran*math.cos(alpha) + roby
    elif (sec == 4 and ort == "R"):
        hd = hdg*(math.pi/180)
        comp = ((yaw*(180/math.pi)) - hdg)
        if comp < 90:
            alpha = abs(yaw - hd)
            xpos = ran*math.cos(alpha) + robx
            ypos = ran*math.sin(alpha) + roby
        else:
            alpha = abs(yaw - (math.pi/2) - hd)
            xpos = (-1)*ran*math.sin(alpha) + robx
            ypos = ran*math.cos(alpha) + roby
    
    check = 1
    print("XPOS = ", xpos)
    print("YPOS = ", ypos)
    
    
def dcallback(msg):
    #Input class of object found
    obj = msg.bounding_boxes[0].Class
    
    global hdg, ort, xcen
    global id, pub, marker
    global xpos, ypos
    #if statement to include desired object, ADD Probability
    if obj == 'person': 
        xmin = msg.bounding_boxes[0].xmin
        xmax = msg.bounding_boxes[0].xmax
        xcen = (xmax - xmin)/2 + xmin
        #video output 1920x720 or 1920x480
        #Field of View 62.2 degrees, horizontal
        #pixel to degree conversion (62.2/1920), horizontal
        
        #force hdg to be an int, round will output float if given float despite being an int
        #heading is center point of bounding box
        #ort used in determining global position of object
    if xcen <= (1920/2):
        ort = "L"
        hdg = int(31 - round(xcen*(62.0/1920.0)))
    else:
       ort = "R"
       hdg = int(360 - round((xcen*(62.0/1920.0)- (1920.0/2.0)*(62.0/1920.0)))

        #Incrament marker, will put duplicate markers for obj until not seen
    
        print("HDG = ",hdg)

    #if statement ensures a proper heading calculated and position calculated
    if check == 1:
        #Marker needs to be related to the base_link to reference the position as measured from the robot
        marker.header.frame_id = "/map"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        #color.a is a transparency setting
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        #all the tf coordinates are rotated -90 from front of robot
        #have to convert the x, y coordinates to the proper system using:
        #x'= xcosT - ysinT, y' = xsinT + ycosT (T = -90)
        marker.pose.position.x = xpos
        marker.pose.position.y = ypos
        marker.pose.position.z = 0.0
        marker.id = id

        #output marker to message
        pub.publish(marker)
        id += 1
        

    


def scanPrint(msg):
        print (len(msg.ranges))

        for i in range (359):
            print(msg.ranges[i])

        #print("rans 0 = ",msg.rans[0])
        #print("ran = ", msg.rans[29])
        #print("rans 90 = ", msg.rans[90])
        #print("rans 180 = ", msg.rans[180])
        #print("rans 270 = ", msg.rans[270])
        #print("rans 360 = ", msg.rans[359])

        #for i in ran(0,360):
        #    print(msg.rans[i])

def Gcallback(msg):
        global m
        global robx, roby
        global xpos, ypos

        #Calculate cell position
    #cell (i,j) -> i = (xpos*N)/a, N is max grid x, a is resolution of cell | j = (ypos*M)/a, M is max grid in y
    
        i = math.floor(((xpos+robx)*384)/.05)
        j = math.floor(((ypos+roby)*384)/.05)
        print("i = ", i)
        print("J = ", j)
       #cell coordinate to list position conversion 
        if i == 0:
            m = int((384*j) + i)
        else:
            m = int((384*j) + i + 1)
        
        print(msg.data[m])

def Ocallback(msg):
        global robx, roby
        global yaw

        robx = msg.pose.pose.position.x
        roby = msg.pose.pose.position.y
        quan = msg.pose.pose.orientation
        quan_list = [quan.x,quan.y,quan.z,quan.w]
        euler = tf.transformations.euler_from_quaternion(quan_list)
        yaw = euler[2]
        #print("YAW = ", yaw*(180/math.pi))
        #print("ROBx = ", robx)
        #print("ROBY = ", roby)



#Main Function

rospy.init_node('adding_markers')
#sub2 = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, dcallback)
test = rospy.Subscriber('/scan', LaserScan, dcallback)
sub = rospy.Subscriber('/scan', LaserScan, Lcallback)
#sub3 = rospy.Subscriber('/map', OccupancyGrid, Gcallback)
sub4 = rospy.Subscriber('/odom', Odometry,  Ocallback )
rospy.spin()
