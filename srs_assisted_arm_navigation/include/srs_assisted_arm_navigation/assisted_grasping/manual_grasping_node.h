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

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"

namespace srs_assisted_arm_navigation {

class ManualGrasping {

public:

    void TactileDataCallback(const std_msgs::Float32MultiArray::ConstPtr & msg);
    void SdhStateCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr & msg);

    ManualGrasping(std::string name)/*:
    server_(nh_, name, boost::bind(&ManualGrasping::execute, this, _1), false)*/
    {

      server_ = new actionlib::SimpleActionServer<ManualGraspingAction>(nh_, name, boost::bind(&ManualGrasping::execute, this, _1), false);

      inited_ = false;
      action_name_ = name;
      jt_publisher_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/sdh_controller/command", 10);
      tact_sub_  = nh_.subscribe("/sdh_controller/mean_values", 10, &ManualGrasping::TactileDataCallback,this);
      state_sub_ = nh_.subscribe("/sdh_controller/state", 10, &ManualGrasping::SdhStateCallback,this);
      server_->start();

    }

    ~ManualGrasping() {

      server_->shutdown();
      delete server_;

    };

    void addJoint(std::string joint);
    void inited(bool val);

protected:

    std::vector<std::string> joints_;
    bool inited_;
    ros::Subscriber tact_sub_;
    ros::Subscriber state_sub_;

    actionlib::SimpleActionServer<ManualGraspingAction> * server_;
    ros::NodeHandle nh_;
    std::string action_name_;

    boost::mutex data_mutex_;

    void execute(const ManualGraspingGoalConstPtr &goal);

    ros::Publisher jt_publisher_;

    std_msgs::Float32MultiArray tactile_data_;
    pr2_controllers_msgs::JointTrajectoryControllerState sdh_data_;

    ros::Time tactile_data_stamp_;

private:


};


} // namespace


#endif /* GRASPING_NODE_H */
