#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from rospy import is_shutdown
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from opcua import Server
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler




#lastpose = []



def poseLogger(msg):
    global lastpose
    lastpose=[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    #orientation_q = msg.pose.pose.orientation
    

def save_possitiion():
    #print(msg.pose.pose)
    #lastpose=[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    print("Saving position...")
    try:
        saved_goals=goals.get_value()
        saved_goals.append(lastpose)
        goals.set_value(saved_goals)
        print("Position is added to the list of goals")
    finally:
        save_goal.set_value(False)
        print(goals.get_value())
        #print("Saving position has failed ")

def reset_goals():
    print("Reseting list of goals")
    goals.set_value([[0,0,0,0]])
    reset.set_value(False)

def exclude_goal():
    goal=exclude.get_value()
    goals_array=goals.get_value()
    if check_goal(goal, goals_array):
        goals_array.pop(goal)
        goals.set_value(goals_array)
        print("Excluding goal")
        
    else: print("Cannot exclude the goal")
    exclude.set_value(0)

def check_goal(goal, saved_goals):
    #print(goal, len(saved_goals))
    if goal>0 and len(saved_goals)>int(goal): 
        return True
    else:
        print("Navigation goal is not in the list") 
        return False               

def start_execution():
    print("Setting the goal")
    
    goals_array=goals.get_value()
    goal=navgoal.get_value()

    if check_goal(goal, goals_array):

        try:
            #rospy.init_node('movebase_client_py')
            result = sendGoal(goals_array[goal])
            if result:
                rospy.loginfo("Goal execution done!")
                location.set_value(goal)
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")
    else:
        start.set_value(False)

def sendGoal(new_goal):
    #print(new_goal)
    print("Starting navigation to goal...")
    
    status.set_value("In process to goal {}".format(new_goal))
    #print(lastpose)
    if (lastpose!=[0,0,0,0]):
        client_nav = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    #print(client_nav.get_state())
    
        client_nav.wait_for_server()
        #print (s)
        #print (float(goal[0]))
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
            status.set_value("Navigation Service is offline")
            print("Navigation Service is offline")
            #rospy.loger("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return False
        else:
            
            status.set_value(client_nav.get_goal_status_text())
            status.set_value("Ready")
            start.set_value(False)
            location.set_value(navgoal.get_value())
            return client_nav.get_result()
    else: 
        print("Navigation service offline")
        status.set_value("Navigation service offline")
    start.set_value(False)

def cancel_execution():
    print("Canceling the Goal")
    cancel.set_value(False)

def set_offset():
    print("Executing offset x = {}, y = {}".format(x.get_value(), y.get_value()))
    position_offset=lastpose[:]
    position_offset[0]+=x.get_value()
    position_offset[1]+=y.get_value()
    
    orientation_list = [0, 0, lastpose[2], lastpose[3]]
    (roll, pitch, rot) = euler_from_quaternion (orientation_list)
    rot+=yaw.get_value()
    #print (roll, pitch, yaw)
    quat = quaternion_from_euler (0, 0,rot)
    position_offset[2], position_offset[3] =  quat[2], quat[3]
    
    
    sendGoal(position_offset)
    pos_offset.set_value(False)
    x.set_value(0)
    y.set_value(0)
    yaw.set_value(0)
    
def setupPoseLogger():    
    rospy.init_node('check_odometry')
    odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, poseLogger)   
    
    try:
        
        print("Starting Server")
        server.start()
        print("Server Online", "Server IP opc.tcp://{}:12345".format(IP))

        while not is_shutdown():
            if start.get_value(): start_execution()
            if cancel.get_value(): cancel_execution()
            if save_goal.get_value(): save_possitiion()
            if reset.get_value(): reset_goals()
            if exclude.get_value()>0 : exclude_goal()
            location_map.set_value(lastpose)
            if pos_offset.get_value(): set_offset()
                
            #print ("State of robot: " + str(status.get_value()))
            #print ("Goals array " + str(goals.get_value()))
        
            rospy.sleep(1)
    #except:
        #print("Something went wrong")    
    finally:
        server.stop()
        print ("Server offline")
        
    
    rospy.sleep(1)
    rospy.spin()


   




if __name__ == '__main__':
    
    lastpose=[0,0,0,0]
    navgoal=[]
    IP=os.environ['ROS_IP']
    server=Server()
    server.set_endpoint("opc.tcp://{}:12345".format(IP))
    server.register_namespace("AGV")

    objects=server.get_objects_node()

    control=objects.add_object('ns=2;s="agv_control"', "Control Inteface")
    
    status=control.add_variable('ns=2;s="status"', "Status_of_robot", "Ready")
    
    navgoal=control.add_variable('ns=2;s="navgoal"', "Navigation goal number", -1)
    navgoal.set_writable()
    
    start=control.add_variable('ns=2;s="start"', "Start execution", False)
    start.set_writable()

    reset=control.add_variable('ns=2;s="reset"', "Reset list of goals", False)
    reset.set_writable()

    save_goal=control.add_variable('ns=2;s="save"', "Save current possition", False)
    save_goal.set_writable()

    goals=control.add_variable('ns=2;s="goals"', "goals_array", [[0,0,0,1]])
    goals.set_writable()

    cancel=control.add_variable('ns=2;s="cancel"', "Status_of_robot", False)
    cancel.set_writable()

    location=control.add_variable('ns=2;s="location"', "current_location", -1)
    location.set_writable()

    exclude=control.add_variable('ns=2;s="exclude"', "current_location", 0)
    exclude.set_writable()

    x=control.add_variable('ns=2;s="x"', "x_location", 0)
    x.set_writable()

    y=control.add_variable('ns=2;s="y"', "y_location", 0)
    y.set_writable()

    yaw=control.add_variable('ns=2;s="yaw"', "yawl roatation in rad", 0)
    yaw.set_writable()

    pos_offset = control.add_variable('ns=2;s="offset"', "Execute possition offset", False)
    pos_offset.set_writable()

    location_map=control.add_variable('ns=2;s="coordinates"', "Coordianates on the map", [0,0,0,0])
    location_map.set_writable()


    
    #global db
    #global table
    #lastpose=[]
    #db = TinyDB(path_resolve(),  cache_size=0)
    #table = db.table('Goals')
    #table_default = db.table('_default')
    
    #print("please enter you command")
    setupPoseLogger()

    
    
    #rospy.init_node('input_test')
    
    