#!/usr/bin/env python


#Location ~/catkin_ws/roboto/scripts2/thingworks.py

#Set LF End of line Sequence

#Requires :
#           goal_management_class.py
#           apierror.py
#           agvcloud.py
#           Running navigation Action Server
#           Running AMCL node



from agvcloud import AGVcloud
import rospy
from goal_managment_class import AGV_management
from geometry_msgs.msg import PoseWithCovarianceStamped
from rospy import is_shutdown


def poseLogger(msg): #log location msg type PoseWithCovarianceStamped[x,y,z,w]
    roboto.location=[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

def check_for_location_name_cloud(h,thing_name,location_name): #queary infotable for GoalName
    [resp, data] = h.get_thing_property(thing_name, 'Goals')
    filtered_data = h.search_in_infotable(data, 'GoalName', location_name, return_column='GoalPosition', print_result=False)
    
    if filtered_data==[]:
        print("Goal {} is not recognized".format(location_name))
        return False
    else:
        print("Goal {} exists in the list".format(location_name))
        return True

def get_next_goal_quat(h,thing_name,location_name): #extract quat coordinates from InfoTable
    [resp, data] = h.get_thing_property(thing_name, 'Goals')

    filtered_data = h.search_in_infotable(data, 'GoalName', location_name, return_column='GoalPosition', print_result=False)
    return [filtered_data["x"],filtered_data["y"],filtered_data["z"],filtered_data["w"]]


if __name__ == "__main__":

    appKey = '93c04eda-3112-4b53-a94b-0060a9e05fe7'
    h = AGVcloud(appKey) # THX handler
    thing_name = 'AGVtest'

    print("Connecting to the server...")

    rospy.init_node('AGV_management_node')
    roboto=AGV_management("Roboto")
    odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, poseLogger) # setup posittion logger




    while not is_shutdown(): #The code needs to be inside, otherwice will be blocked
        
        [resp, data] = h.get_thing_property(thing_name, 'SaveLocation', print_result=False)

        if data: 
            
            resp = h.put_thing_property(thing_name, 'SaveLocation', "False", print_result=True, show_old_value=False)
            roboto.save_location_name= h.get_thing_property(thing_name, 'SaveLocationName', print_result=False)[1]
            
            print(roboto.save_location_name)
            
            
            if check_for_location_name_cloud(h,thing_name,roboto.save_location_name):
                resp = h.put_in_infotable(thing_name, 'Goals', 'GoalName', roboto.save_location_name, 'GoalPosition', {'x': roboto.location[0], 'y': roboto.location[1], 'z': roboto.location[2], 'w':roboto.location[3]}, print_result=True, show_old_value=False)
            else:
                resp = h.put_thing_property(thing_name, 'Debug', "Save location failed! Name is not in the list", print_result=False, show_old_value=False)

        [resp, data] = h.get_thing_property(thing_name, 'Execute', print_result=False)

        if data:
            resp = h.put_thing_property(thing_name, 'Execute', "False", print_result=True, show_old_value=False)
            [resp, data] = h.get_thing_property(thing_name, 'NextGoalName', print_result=False)
            roboto.next_goal_name=data
            if check_for_location_name_cloud(h,thing_name, roboto.next_goal_name): #Check if location name exist in infotable "Goals"
                roboto.next_goal=get_next_goal_quat(h,thing_name,roboto.next_goal_name) #Set next_goal_quaterniun 
                roboto.start_execution() #Send goal quat to Navigation Action (can be replaced with go_to(pointer.next_goal))
            else:
                resp = h.put_thing_property(thing_name, 'Debug', "Goal is not found", print_result=False, show_old_value=False)
        resp = h.put_thing_property(thing_name, 'Status', roboto.status, print_result=False, show_old_value=False) #set robot Status (Location_name or Fail)



        rospy.sleep(1)