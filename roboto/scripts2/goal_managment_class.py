#!/usr/bin/env python


#Location ~/catkin_ws/roboto/scripts2/goal_management_class.py

#Set LF End of line Sequence

#Requires: Running Navigation Action Server

import rospy
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from rospy import is_shutdown





class AGV_management:


    
    def __init__(self, name):
        
        self.name=name
        self.status = "Online"
        self.location=[0,0,0,0]
        self.next_goal=[0,0,0,0]
        self.stored_positions=[]
        self.save_location=False
        self.save_location_name=""
        self.next_goal_name=""
        
        
        self.offset_x=0
        self.offset_y=0
        self.offset_yaw=0

    
    def goto_position(self, new_goal): #access Navigation ActionServer
        print("Starting navigation to goal...")
        self.status = "In process to goal {}".format(self.next_goal_name)
        

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
            print("Goal is send")
            wait = client_nav.wait_for_result()
        
            if not wait:
                #self.status = "Navigation Service is offline"
                print("Navigation Service is offline")
                self.status = "Fail"
                
            else:
                
                self.status=client_nav.get_goal_status_text()
                self.status = self.next_goal_name
                print("Goal is accepted")
                
                
        else: 
            print("Navigation service offline")
            self.status = "Fail"

    def start_execution(self):

       self.goto_position(self.next_goal)


    def set_offset(self): #not implemented
        
        print("Executing offset x = {}, y = {}".format(self.offset_x, self.offset_y))
        position_offset=self.location[:]
        position_offset[0]+=self.offset_x
        position_offset[1]+=self.offset_y
        
        orientation_list = [0, 0, self.location[2], self.location[3]]
        (roll, pitch, rot) = euler_from_quaternion (orientation_list)
        rot+= self.offset_yaw
        quat = quaternion_from_euler (0, 0,rot)
        position_offset[2], position_offset[3] =  quat[2], quat[3]
        
        
        self.goto_position(position_offset)
        
        self.offset_x=0
        self.offset_y=0
        self.offset_yaw=0
        


        
    


