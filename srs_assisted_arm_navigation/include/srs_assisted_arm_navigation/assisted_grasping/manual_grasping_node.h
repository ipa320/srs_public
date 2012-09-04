/******************************************************************************
 * \file manual_grasping_node.h
 * \brief Node for manual grasping
 * \author Zdenek Materna (imaterna@fit.vutbr.cz)
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Zdenek Materna (imaterna@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd/mm/2012
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * This is node which acts as bridge between functionality of warehouse viewer and RVIZ plugin.
 * It provides actionlib interface (server) for communication with i.e. decision making and some services for
 * communication with RVIZ arm manipulation plugin.
 *
 */

#pragma once
#ifndef GRASPING_NODE_H
#define GRASPING_NODE_H

#include <ros/ros.h>
#include <string>
#include <math.h>

#include <actionlib/server/simple_action_server.h>

#include "srs_assisted_arm_navigation/services_list.h"
#include "srs_assisted_arm_navigation/topics_list.h"

#include "srs_assisted_arm_navigation/ManualGraspingAction.h"
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include <actionlib/client/simple_action_client.h>
#include "cob_srvs/SetOperationMode.h"
//#include "brics_actuator/JointVelocities.h"

/*#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>*/

namespace srs_assisted_arm_navigation {


class ManualGrasping {

public:

    void TactileDataCallback(const std_msgs::Float32MultiArray::ConstPtr & msg);
    void SdhStateCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr & msg);

    bool OpenGripper();

    void waitForNewData(double rate, std_msgs::Float32MultiArray& tactile_data, pr2_controllers_msgs::JointTrajectoryControllerState& sdh_data);

    void PublishVelocities();
    void stopMotion();

    bool setPosition(std::vector<double> positions);

    ManualGrasping(std::string name)/*:
    server_(nh_, name, boost::bind(&ManualGrasping::execute, this, _1), false)*/
    {

      server_ = new actionlib::SimpleActionServer<ManualGraspingAction>(nh_, name, boost::bind(&ManualGrasping::execute, this, _1), false);

      inited_ = false;

      tact_received_ = false;
      sdh_received_ = false;

      /*std::vector<brics_actuator::JointValue> jv;

      for (unsigned int i=0;i<joints_.size();i++) {

    	  brics_actuator::JointValue jvt;

    	  jvt.joint_uri = joints_[i];
    	  jvt.unit = boost::units::to_string(boost::units::si::radians);
    	  jvt.value = 0.0;

    	  jv.push_back();

      }*/

      sdh_mode_client_ = nh_.serviceClient<cob_srvs::SetOperationMode>("/sdh_controller/set_operation_mode");

      rate_ = 20.0;

      min_contact_force_ = 20;

      max_speed_ = 3.66/10; // max. vel. 210 deg / sec -> 3.66 rad / s
      accel_ = max_speed_ / (1.0 * rate_); // accelerate to max speed in xy seconds


      //vel_publisher_ = nh_.advertise<brics_actuator::JointVelocities>("/sdh_controller/command_vel", 10);
      vel_publisher_ = nh_.advertise<std_msgs::Float32MultiArray>("/sdh_controller/set_velocities_raw", 10);

      sdh_trajectory_ctrl_client_ = new tsdh_trajectory_ctrl_client("/sdh_controller/follow_joint_trajectory",true);

      action_name_ = name;
      //jt_publisher_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/sdh_controller/command", 10);
      tact_sub_  = nh_.subscribe("/sdh_controller/mean_values", 10, &ManualGrasping::TactileDataCallback,this);
      state_sub_ = nh_.subscribe("/sdh_controller/state", 10, &ManualGrasping::SdhStateCallback,this);
      server_->start();

      vel_.data.push_back(0.0);
      vel_.data.push_back(0.0);
      vel_.data.push_back(0.0);
      vel_.data.push_back(0.0);
      vel_.data.push_back(0.0);
      vel_.data.push_back(0.0);
      vel_.data.push_back(0.0);

      /*positions_.push_back(0.0);
      positions_.push_back(-0.9854);
      positions_.push_back(0.7472);
      positions_.push_back(-0.9854);
      positions_.push_back(0.7472);
      positions_.push_back(-0.9854);
      positions_.push_back(0.7472);

      velocities_.push_back(0.003);
      velocities_.push_back(0.003);
      velocities_.push_back(0.003);
      velocities_.push_back(0.003);
      velocities_.push_back(0.003);
      velocities_.push_back(0.003);
      velocities_.push_back(0.003);*/

    }

    ~ManualGrasping() {

      server_->shutdown();
      delete server_;

    };

    void addJoint(std::string joint);
    void inited(bool val);

    bool setMode(std::string mode);

protected:

    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> tsdh_trajectory_ctrl_client;

    tsdh_trajectory_ctrl_client *sdh_trajectory_ctrl_client_;

    //brics_actuator::JointVelocities vel_;
    std_msgs::Float32MultiArray vel_;
    ros::Publisher vel_publisher_;

    ros::ServiceClient sdh_mode_client_;


    std::vector<std::string> joints_;
    bool inited_;
    ros::Subscriber tact_sub_;
    ros::Subscriber state_sub_;

    bool tact_received_;
    bool sdh_received_;

    actionlib::SimpleActionServer<ManualGraspingAction> * server_;
    ros::NodeHandle nh_;
    std::string action_name_;

    boost::mutex data_mutex_;

    bool initialized_;

    float min_contact_force_;

    float max_force_;
    //float inc_;

    float max_speed_;
    float accel_;

    float rate_;

    void execute(const ManualGraspingGoalConstPtr &goal);

    //bool CloseRound(std::vector<double>& pos, std_msgs::Float32MultiArray& tact, uint8_t pos_t, uint8_t pos_f, uint8_t tact_t, uint8_t tact_f);
    //bool CloseSquare(std::vector<double>& pos, std_msgs::Float32MultiArray& tact, uint8_t pos_t, uint8_t pos_f, uint8_t tact_t, uint8_t tact_f);
   // bool InitializeFinger(std::vector<double>& pos,uint8_t pos_t, uint8_t pos_f);

    //bool SetPhalanxPosition(std::vector<double>& pos,uint8_t idx, double value, double inc);

    ros::Publisher jt_publisher_;

    std_msgs::Float32MultiArray tactile_data_;
    pr2_controllers_msgs::JointTrajectoryControllerState sdh_data_;

    ros::Time tactile_data_stamp_;

    std::vector<double> positions_;
    std::vector<double> velocities_;

private:


};


} // namespace


#endif /* GRASPING_NODE_H */
