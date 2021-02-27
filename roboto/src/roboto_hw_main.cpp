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

/* Author: Yevheniy Dzezhyts
   Desc:   Example ros_control main() entry point for controlling robots in ROS
*/

#include <ros/ros.h>
#include <roboto/generic_hw_control_loop.h>
#include <roboto/roboto_hw_interface.h>
#include <std_msgs/Int32MultiArray.h>
#include "std_msgs/Int32.h"




 

int main(int argc, char** argv)
{
 
   
  //Initiate hardware interface node
  ros::init(argc, argv, "roboto_hw_interface");
  ros::NodeHandle nh;

  //Share pointer to RobotoHWInterdface class
  boost::shared_ptr<roboto_control::RobotoHWInterface> roboto_hw_interface
    (new roboto_control::RobotoHWInterface(nh));
  
   //&roboto_control::RobotoHWInterface::num_joints_=2
  

  //Setup publisher to send commands 
  ros::Publisher commander = nh.advertise<std_msgs::Int32MultiArray>("commander", 1000);
  
  
  //Set commander inside of class
  roboto_hw_interface->init_talker(commander);
  
  
  
 
    //Setup subscribers for left and right wheel
    

      
    //ros::Subscriber left_sub = nh.subscribe("/"+roboto_hw_interface->get_joint_names()[0], 1000, &roboto_control::RobotoHWInterface::leftWheelPos, roboto_hw_interface);
    //#ros::Subscriber right_sub = nh.subscribe("/"+roboto_hw_interface->get_joint_names()[1], 1000, &roboto_control::RobotoHWInterface::rightWheelPos, roboto_hw_interface);

    ros::Subscriber left_sub = nh.subscribe("/lfwheel", 1000, &roboto_control::RobotoHWInterface::leftWheelPos, roboto_hw_interface);
    ros::Subscriber right_sub = nh.subscribe("/rfwheel", 1000, &roboto_control::RobotoHWInterface::rightWheelPos, roboto_hw_interface);
    
    
    
  //We need asyncronious spinner to avoid other notes from blocking
  ros::AsyncSpinner spinner(3);
  spinner.start();
    

   
  
  
  //Initiate hardware interface
  roboto_hw_interface->init();
  
 
  
  
  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, roboto_hw_interface);
  control_loop.run(); // Blocks until shutdown signal recieved

  return 0;
}



