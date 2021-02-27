#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;
  //double pi=3.141592653589793238;
  std::string base_link;
  std::string frame_id;

  double pos_x;
  double pos_y;
  double pos_z;
  
  double roll;
  double pitch;
  double yaw;
  
  
  ros::NodeHandle nh_private("~");
  nh_private.param<std::string>("base_link", base_link, "chassis");
  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
  nh_private.param<double>("pos_x", pos_x, 0.0);
  nh_private.param<double>("pos_y", pos_y, 0.0);
  nh_private.param<double>("pos_z", pos_z, 0.0);
 
  nh_private.param<double>("roll", roll, 0.0);
  nh_private.param<double>("pitch", pitch, 0.0);
  nh_private.param<double>("yaw", yaw, 0.0);
  


  tf::Quaternion myQuaternion;
  myQuaternion.setRPY( roll, pitch, yaw);
  myQuaternion.normalize();
  //tf::Vector3(-0.40, 0, 0.44))
  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(myQuaternion, tf::Vector3(pos_x, pos_y, pos_z)),
        ros::Time::now(),base_link, frame_id));
    r.sleep();
  }
}
