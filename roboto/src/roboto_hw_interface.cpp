/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Skovde, Sweden
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Yevheniy Dzehyts
   Desc:   This module is based on ros_control boilerplate by Dave Coleman. 
          The roboto_hw_interface holds read and right functions fo  Roboto.
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <roboto/roboto_hw_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>



namespace roboto_control
{

  ros::Subscriber left_sub;
  ros::Subscriber right_sub;
 

RobotoHWInterface::RobotoHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)

{
 
  ROS_INFO_NAMED("roboto_hw_interface", "RobotoHWInterface Ready.");
  // Load rosparams
  ros::NodeHandle rpnh(nh_, "hardware_interface");
  std::size_t error = 0;
  // Next parameters are set by .yaml file
  error += !rosparam_shortcuts::get(name_, rpnh, "sim_control_mode", sim_control_mode_); // to disable sim mode set sim_control_mode to 0
  error += !rosparam_shortcuts::get(name_, rpnh, "ticks_per_revolution", ticks_per_revolution);  //ticks pere full revolution
  error += !rosparam_shortcuts::get(name_, rpnh, "signal_multiplier", signal_multiplier);
  //error += !rosparam_shortcuts::get(name_, rpnh, "joint_topics", joint_topics); 

    ros::Subscriber left_sub = nh.subscribe("/lfwheel", 1000, &RobotoHWInterface::leftWheelPos, this);
    ros::Subscriber right_sub = nh.subscribe("/rfwheel", 1000, &RobotoHWInterface::rightWheelPos, this);
  if (error)
  {
    ROS_WARN_STREAM_NAMED(name_, "SimHWInterface now requires the following config in the yaml:");
    ROS_WARN_STREAM_NAMED(name_, "   sim_control_mode: 0 # 0: position, 1: velocity");
  }
  rosparam_shortcuts::shutdownIfError(name_, error);

}

ros::Publisher commander;




int RobotoHWInterface::get_sim_mode(){
  return sim_control_mode_;
}

std::vector<std::string> RobotoHWInterface::get_joint_names(){
  return joint_names_;
}


void RobotoHWInterface::init_talker(ros::Publisher commander_){
  commander=commander_;

}


//-------------------ODOMETER READ BLOCK------------------------
void RobotoHWInterface::getPos(int joint_id, double value){
  joint_position_[joint_id]=value;
  
  //ROS_INFO("I heard: [%f]", joint_position_[joint_id]);  
  }
void RobotoHWInterface::leftWheelPos(const std_msgs::Int32ConstPtr& msg){
  getPos(0, double(msg->data)/(ticks_per_revolution/6.28));

}
void RobotoHWInterface::rightWheelPos(const std_msgs::Int32ConstPtr& msg){
  getPos(1, double(msg->data)/(ticks_per_revolution/6.28));

}
//------------------END OF ODOMETER BLOCK------------------------------  


void RobotoHWInterface::read(ros::Duration &elapsed_time) 
{
  //This code does not require read function as read will be done by callback function
  //from lfwheel and rfwheel topics
  
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR READ COMMAND FROM USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void RobotoHWInterface::write(ros::Duration &elapsed_time)
{
  
if (sim_control_mode_==0){
  // Safety
  enforceLimits(elapsed_time);
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
  // VELOCITY FROM POSITION WITH SMOOTHING, SEE
  // sim_hw_interface.cpp IN THIS PACKAGE
  //
  // DUMMY PASS-THROUGH CODE
  
  msg.data.clear(); //clear message 

  for (std::size_t joint_id = 0; joint_id <num_joints_; ++joint_id){ //Loop through joints
    
    
    msg.data.push_back (int(joint_velocity_command_[num_joints_-joint_id-1]*signal_multiplier)); //Prepair command for publishing
    
    }

      commander.publish(msg); //Publish command to topic /commander
  }
else{

    enforceLimits(elapsed_time);
    for (std::size_t joint_id = num_joints_; joint_id >-1; --joint_id){ //Loop through joints
    joint_position_[joint_id] += joint_velocity_command_[joint_id] * elapsed_time.toSec();  
    
    }

  // END DUMMY CODE FOR SIMULATION MODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  }
}

void RobotoHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}








}  // namespace
