#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from rospy import is_shutdown
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys
import os, os.path
from tinydb import TinyDB, where, Query

#lastpose = []

def path_resolve(): #Resolve path to /home/user/var_db.json
	f_path=sys.path[0].split("/")
	return os.path.join("/", f_path[1],f_path[2], "var_db.json")

def add_value(name,value): #add or edit record
	db= TinyDB(path_resolve())
	if (db.get(where('name') == name)==None):
		db.insert({'name': name, 'value':value})
	else:
		db.update({'value': value}, where('name') == name)
		#db.insert({'name': name, 'value':value})
	db.close()

def remove(field, value, table):
	db=TinyDB(path_resolve())
	if table:
		tb=db.table('Goals')
		tb.remove(where(field) == value)
	else:
		db.remove(where(field) == value)
	db.close()

def read(field, value, table):
	db= TinyDB(path_resolve())
	if table:
		tb=db.table('Goals')
		result=tb.get(where(field) == value)
	else: 
		result=db.get(where(field) == value) 
	db.close()
	return result

def readall():
	db= TinyDB(path_resolve())
	tb=db.table('Goals')
	return tb.all()
	db.close()
def insertgoal(x,y,z,w,visual):
	db= TinyDB(path_resolve())
	tb=db.table('Goals')
	tb.insert({'x':x, 'y':y, 'z':z, 'w':w, 'Visual': visual})
	db.close()


def poseLogger(msg):
    global lastpose
    lastpose=[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

def saveAsGoal(visual):
    #print(msg.pose.pose)
    #lastpose=[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    print("Saving goal...")
    
    #if (db.get(where('name') == 'SaveGoal' is not None):
    if(len(lastpose))>0:
        
        insertgoal(lastpose[0], lastpose[1], lastpose[2], lastpose[3], visual['value'])
        #table.insert({ 'x':lastpose[0], 'y':lastpose[1], 'z':lastpose[2], 'w':lastpose[3], 'Visual': visual['value']})
        return True
    else: return False
           

def setGoal(goal):
    print("Setting the goal {}....".format(goal))
    goals=readGoals()
    #sendGoal(goals[int(goal)])

    try:
        #rospy.init_node('movebase_client_py')
        result = sendGoal(goals[int(goal)])
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

def sendGoal(new_goal):
    #print(new_goal)
    print("Starting navigation to goal...")
    
    add_value('Status','Naviagating to goal '+new_goal['Visual'])
    add_value('RobotStatus','Busy')

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    #print(client.get_state())
    client.wait_for_server()
    #print (s)
    #print (float(goal[0]))
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(new_goal['x'])
    goal.target_pose.pose.position.y = float(new_goal['y'])
    goal.target_pose.pose.orientation.z = float(new_goal['z'])
    goal.target_pose.pose.orientation.w = float(new_goal['w'])
    
    client.send_goal(goal)
    print(client.get_goal_status_text())
    wait = client.wait_for_result()
   
    if not wait:
        add_value('Status','Naviagation service not avaliable')
        add_value('RobotStatus','NotReady')
        rospy.loger("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        add_value('Status', client.get_goal_status_text())
        
        #print(client.get_goal_status_text())
        add_value('LastPosition', new_goal['Visual'])
        add_value('RobotStatus','Ready')
        return client.get_result()

    
def cancelGoal():
    print("Canceling the Goal")

    
def setupPoseLogger():    
    rospy.init_node('check_odometry')
    odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, poseLogger)
    
    while not is_shutdown():
        check_command()
        rospy.sleep(1)
    rospy.sleep(1)
    rospy.spin()

   

def check_command():

    saved_goal = read('name','SaveGoal', False)
    goto_goal = read('name', 'Goto', False)
    cancel_goal= read('name', 'Cancel', False)
    #client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    #print(client.get_result())
    #print goto_goal
    if (saved_goal is not None):
        print("goal recieved")
        if saveAsGoal(saved_goal):
            #db.remove(where('name') == 'SaveGoal')
            remove('name','SaveGoal', False)
    
    elif(goto_goal):
        sendGoal(read('Visual',goto_goal['value'],True))
        remove('name','Goto', False)
        #db.remove(where('name') == 'Goto')
    
    elif(cancel_goal):
        cancelGoal()
        remove('name','Cancel',False)
        #db.remove(where('name') == 'Cancel')
    #db.close()    
def get_status():
    #TODO
    return "TODO"

def update_status():
    add_value("Status", get_status, db)

if __name__ == '__main__':
    #global db
    #global table
    #lastpose=[]
    #db = TinyDB(path_resolve(),  cache_size=0)
    #table = db.table('Goals')
    #table_default = db.table('_default')
    
    #print("please enter you command")
    setupPoseLogger()

    
    
    #rospy.init_node('input_test')
    
    