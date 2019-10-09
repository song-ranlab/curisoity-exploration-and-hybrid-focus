#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from curious_obj_routine.msg import curious

x = 100
y = 100


def movebase_client(x,y):

    #create action client, with MoveBaseAction
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    #Wait until action server has started
    client.wait_for_server()

    #Create new goals:
    #Goal setup parameters
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    #Goal Coordinates
    goal.target_pose.pose.position.x = x - 0.5
    #Goal orientation w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

    
    '''
    #Send goal to action server
    client.send_goal(goal)
    #Wait for action to be preformed
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not avialable")
    else:
        return client.get_result()
    '''

def listen_cb(msg):
    global x, y    
    x = msg.x
    y = msg.y
    #print x, y




#Main
if __name__ == '__main__':
    #initialize node
    rospy.init_node('movebase_client')

    sub = rospy.Subscriber('coord', curious, listen_cb)
    movebase_client(x,y)

    rospy.spin()