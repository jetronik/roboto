#!/usr/bin/env python

import rospy
import actionlib

from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

def robot_pos_tracking(data):
    x= data.pose.pose.position.x
    y= data.pose.pose.position.y
    z=data.pose.pose.orientation.z
    w=data.pose.pose.orientation.w
    transfer_odom(x,y,z,w)

def transfer_odom(x,y,z,w):
    #rospy.init_node('set_pose')

    state_msg = ModelState()
    state_msg.model_name = 'Roboto'
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = 0.0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = z
    state_msg.pose.orientation.w = w

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e




if __name__ == '__main__':

    rospy.init_node('model_state_gazebo')
    
    try:
        odom_sub = rospy.Subscriber('/diff_drive_controller/odom', Odometry, robot_pos_tracking)
    except rospy.ROSInterruptException:
        pass

    rospy.spin()