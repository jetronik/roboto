#!/usr/bin/env python

import rospy
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from rospy import is_shutdown





class AGV_management:


    
    def __init__(self, name):
        
        self.name=name
        self.status = "Init"
        self.location=[0,0,0,0]
        self.next_goal=[0,0,0,0]
        self.stored_positions=[]
        
        
        self.offset_x=0
        self.offset_y=0
        self.offset_yaw=0

    
    def goto_position(self, new_goal):
        print("Starting navigation to goal...")
    
        self.status = "In process to goal {}".format(new_goal)
        #print(lastpose)
        if (self.location!=[0,0,0,0]):
            client_nav = actionlib.SimpleActionClient('move_base',MoveBaseAction)
            client_nav.wait_for_server()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = float(new_goal[0])
            goal.target_pose.pose.position.y = float(new_goal[1])
            goal.target_pose.pose.orientation.z = float(new_goal[2])
            goal.target_pose.pose.orientation.w = float(new_goal[3])
            
            client_nav.send_goal(goal)
            print(client_nav.get_goal_status_text())
            wait = client_nav.wait_for_result()
        
            if not wait:
                self.status = "Navigation Service is offline"
                print("Navigation Service is offline")
                #rospy.loger("Action server not available!")
                #rospy.signal_shutdown("Action server not available!")
                return False
            else:
                
                self.status=client_nav.get_goal_status_text()
                self.status = "Ready"
                
                #location.set_value(navgoal.get_value())
                return True
        else: 
            print("Navigation service offline")
            self.status = "Navigation service offline"

    def start_execution(self):

        self.goto_position(self.next_goal)


    def set_offset(self):
        
        print("Executing offset x = {}, y = {}".format(self.offset_x, self.offset_y))
        position_offset=self.location[:]
        position_offset[0]+=self.offset_x
        position_offset[1]+=self.offset_y
        
        orientation_list = [0, 0, self.location[2], self.location[3]]
        (roll, pitch, rot) = euler_from_quaternion (orientation_list)
        rot+= self.offset_yaw
        #print (roll, pitch, yaw)
        quat = quaternion_from_euler (0, 0,rot)
        position_offset[2], position_offset[3] =  quat[2], quat[3]
        
        
        self.goto_position(position_offset)
        #pos_offset.set_value(False)
        self.offset_x=0
        self.offset_y=0
        self.offset_yaw=0
        


        
    


