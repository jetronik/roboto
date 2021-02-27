/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the Roboto
           For a more detailed simulation example, see sim_hw_interface.h
*/

#ifndef ROBOTO_CONTROL__ROBOTO_HW_INTERFACE_H
#define ROBOTO_CONTROL__ROBOTO_HW_INTERFACE_H

#include <roboto/generic_hw_interface.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"



namespace roboto_control
{


/// \brief Hardware interface for a robot
class RobotoHWInterface : public ros_control_boilerplate::GenericHWInterface
{

 




public:


  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  RobotoHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

   // Send commands in different modes
  int sim_control_mode_ = 0;
  std::vector<double> temp_joint_position_;
  std_msgs::Int32MultiArray msg;
  //ros::Subscriber left_sub;
  //ros::Subscriber right_sub;
 
  //ros::NodeHandle nh;

  virtual int get_sim_mode();
  virtual std::vector<std::string> get_joint_names();
  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration &elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration &elapsed_time);


  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration &period);
  
  virtual void init_talker(ros::Publisher commander_);

  //virtual void init_sim(ros::Publisher lfwheel_, ros::Publisher rfwheel_);

  //virtual void setup_subscribers(ros::NodeHandle nh);  

  //virtual void sim_pub(ros::Duration elapsed_time);

   virtual void rightWheelPos( const std_msgs::Int32ConstPtr& msg);

  virtual void leftWheelPos( const std_msgs::Int32ConstPtr& msg);
 
  
  //void operator()(const std_msgs::StringConstPtr& message)

  //virtual void publish_command( std::vector<double> command);

 //virtual void setPos(int joint_id, double value);

 virtual void getPos(int joint_id, double value);

  virtual double getVelocity(int joint_id){
    
      return  joint_velocity_command_[joint_id];
  }
  

};  // class







}  // namespace


#endif
