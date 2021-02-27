from agvcloud import AGVcloud
import goal_managment_class



if __name__ == "__main__":

    appKey = '93c04eda-3112-4b53-a94b-0060a9e05fe7'
    h = AGVcloud(appKey) # THX handler

    def poseLogger(msg):
        roboto.location=[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

    rospy.init_node('AGV_management_node')
    roboto=AGV_management("Roboto")
    odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, poseLogger)




while not is_shutdown():
    print(roboto.name)
    #roboto.next_goal= [1,1,1,1]
    #roboto.offset_y=1
    #roboto.set_offset()
    print(roboto.status)

    
    
    
    
    try:
        [resp, data] = h.get_thing_property('AGVtest', 'Status', print_result=True)
    except:
        print("Something went wrong!")


    # Get current state of automated variable 'CubeVisible'
    # [resp, data] = h.get_thing_property('CabinetTest', 'CubeVisible', print_result=True)

    # Invert 'RedButton' variable to make 'CubeVisible' change
    resp = h.put_thing_property('AGVtest', 'Status', "Online", print_result=True, show_old_value=True)

    # Retrieve automated variable 'CubeVisible' back
    # [resp, data] = h.get_thing_property('CabinetTest', 'CubeVisible', print_result=True)
    rospy.sleep(1)