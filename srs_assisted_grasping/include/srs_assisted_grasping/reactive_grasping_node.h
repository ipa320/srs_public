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
#include <actionlib/client/simple_action_client.h>

#include "srs_assisted_grasping/services_list.h"
#include "srs_assisted_grasping/topics_list.h"

#include "srs_assisted_grasping_msgs/ReactiveGraspingAction.h"
//#include <control_msgs/FollowJointTrajectoryAction.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"

#include "cob_srvs/SetOperationMode.h"
#include "brics_actuator/JointVelocities.h"
#include "schunk_sdh/TactileSensor.h"

#include "XmlRpcValue.h"

/*#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>*/

namespace srs_assisted_grasping {

struct ReactiveGraspingParams {

    ros::Duration default_time;
    double max_force;
    double max_velocity;
    double a_ramp;
    double d_ramp;
    double rate;

};

struct Joints {


	std::vector<std::string> joints;
	std::vector<bool> has_tactile_pad;
	std::vector<bool> is_static;

	int num_of_tactile_pads;

	std::vector< std::vector<double> > velocities;

};

// accessed from callbacks
struct FeedbackData {

	//schunk_sdh::TactileSensor tactile_data;
	std::vector<int16_t> tactile_data;
	pr2_controllers_msgs::JointTrajectoryControllerState sdh_data;

	ros::Time tactile_data_stamp;
	ros::Time sdh_data_stamp;

	bool sdh_data_checked;

	bool tactile_data_checked;

	boost::mutex data_mutex;

};


class ReactiveGrasping {

public:

    //void TactileDataCallback(const std_msgs::Float32MultiArray::ConstPtr & msg);
	void TactileDataCallback(const schunk_sdh::TactileSensor::ConstPtr & msg);
    void SdhStateCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr & msg);



    ReactiveGrasping(std::string name);
    ~ReactiveGrasping(void);



protected:

    bool fatal_error_;

    bool readParams();

    ros::ServiceClient sdh_mode_client_; // for setting SDH to right mode

    ros::Subscriber tact_sub_; // for receiving tactile data
    ros::Subscriber state_sub_; // for receiving sdh state data


    actionlib::SimpleActionServer<srs_assisted_grasping_msgs::ReactiveGraspingAction> * server_;

    ros::NodeHandle nh_;
    std::string action_name_;




    void execute(const srs_assisted_grasping_msgs::ReactiveGraspingGoalConstPtr &goal);

    ros::Publisher vel_publisher_;

    bool publish(std::vector<double> vel);
    void stop();


    ReactiveGraspingParams params_;
    Joints joints_;
    FeedbackData feedback_data_;

    bool setMode(std::string mode);

    /*std_msgs::Float32MultiArray tactile_data_;
    pr2_controllers_msgs::JointTrajectoryControllerState sdh_data_;*/

    void copyData();


    // these variable are accessed only from execute callback thread
    //schunk_sdh::TactileSensor tactile_data_;
    std::vector<int16_t> tactile_data_;
    trajectory_msgs::JointTrajectoryPoint sdh_data_act_;

    std::vector<float> time_to_stop_;

private:




};


} // namespace


#endif /* GRASPING_NODE_H */
