#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from rospy import is_shutdown
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#lastpose = []

def poseLogger(msg):
    global lastpose
    lastpose=[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
def saveAsGoal():
    #print(msg.pose.pose)
    #lastpose=[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    print("Saving goal...")
    s=lastpose
    
    print(lastpose)
    with open("target.csv", "a") as f:
    
        f.writelines(["%s " % item  for item in s])
        f.writelines("\n")
    f.close()    

def readGoals():
    
    goals=[]
    print("Reading goals...s")
    with open("target.csv", "r") as f:
        k=f.readlines()
        for i, l in enumerate(k):
            goals.append(l.split())
            
        #print(k)
        #for line, linenum in enumerate(text_file.split("#"))
        #    print ("Goal %s", linenum)
    f.close()
    return goals

def displayGoals():
    goals=readGoals()
    for i, l in enumerate(goals):
            print("Goal {}".format(i))
            print(l)

def cleanGoals(goalIndex):
    if (goalIndex=="all"):
        #delet the file
        print("todo")
    else: 
        print("todo")#delete line in file

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
    print("Sending the goal....")
    
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    #print (goal)
    #print (float(goal[0]))
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(new_goal[0])
    goal.target_pose.pose.position.y = float(new_goal[1])
    goal.target_pose.pose.orientation.w = float(new_goal[2])
    goal.target_pose.pose.orientation.w = float(new_goal[3])

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

    
def cancelGoal():
    print("Canceling the Goal")
#def listen(msg):
   

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/goal_manager", String, listen)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



    
def setupPoseLogger():    
    rospy.init_node('check_odometry')
    odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, poseLogger)
    

    while not is_shutdown():
        print("please enter you command:")
        x=raw_input()
        
        if (x=="save"):
            saveAsGoal()
        elif(x=="read"):
            displayGoals()
        elif(x=="clean"):
            cleanGoals()
        elif(x[0:4]=="goto"):
            setGoal(x[5:6])
        elif(x=="cancel"):
            cancelGoal()
        else: print("Unknown command")
    
    rospy.spin()

if __name__ == '__main__':
    
    #print("please enter you command")
    setupPoseLogger()
    
    #rospy.init_node('input_test')
    
    