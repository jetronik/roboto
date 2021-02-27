import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    print (msg.pose.pose)

rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/amcl_pos', Odometry, callback)

rospy.spin()

