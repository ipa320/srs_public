/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Zdenek Materna (imaterna@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 5/4/2012
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

#include "srs_assisted_arm_navigation_ui/state_vis.h"

using namespace srs_assisted_arm_navigation_ui;

StateVis::StateVis() {

	// TODO show "Set goal position" at beginning and after first interaction "Position is ok" ???

	ros::NodeHandle nh;

	color_def_.r = 0.2;
	color_def_.g = 0.4;
	color_def_.b = 1.0;
	color_def_.a = 1.0;

	color_err_.r = 1.0;
	color_err_.g = 0.42;
	color_err_.b = 0.42;
	color_err_.a = 1.0;

	m_.header.frame_id = "/rviz_cam_add";
	m_.header.stamp = ros::Time::now();
	m_.ns = "state_vis";
	m_.id = 0;
	m_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	m_.action = visualization_msgs::Marker::ADD;
	m_.pose.position.x = 0.0;
	m_.pose.position.y = 0.0;
	m_.pose.position.z = 0.8;
	m_.pose.orientation.x = 0.0;
	m_.pose.orientation.y = 0.0;
	m_.pose.orientation.z = 0.0;
	m_.pose.orientation.w = 1.0;
	m_.scale.z = 0.10; // specifies the height of an uppercase "A".
	m_.color = color_def_;
	m_.text = "";
	m_.lifetime = ros::Duration(1.0);

	j_.header.frame_id = "/sdh_palm_link";
	j_.header.stamp = ros::Time::now();
	j_.ns = "state_vis";
	j_.id = 1;
	j_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	j_.action = visualization_msgs::Marker::ADD;
	j_.pose.position.x = 0.0;
	j_.pose.position.y = 0.0;
	j_.pose.position.z = 0.8;
	j_.pose.orientation.x = 0.0;
	j_.pose.orientation.y = 0.0;
	j_.pose.orientation.z = 0.0;
	j_.pose.orientation.w = 1.0;
	j_.scale.z = 0.10; // specifies the height of an uppercase "A".
	j_.color = color_err_;
	j_.text = "Joints out of limits!";
	j_.lifetime = ros::Duration(1.0);

	marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/but_arm_manip/state_vis",1);

	// add markers...
	/*visualization_msgs::MarkerArray arr;
	arr.markers = markers_;
	marker_pub_.publish(arr);*/

	// from now we will just modify markers...
	//markers_[0].action = visualization_msgs::Marker::MODIFY;

	arm_nav_state_sub_ = nh.subscribe<srs_assisted_arm_navigation_msgs::AssistedArmNavigationState>("/but_arm_manip/state",1, &StateVis::stateCallback,this);

	ROS_INFO("Assisted arm navigation visualization initialized.");

}

StateVis::~StateVis() {



}


void StateVis::stateCallback(const srs_assisted_arm_navigation_msgs::AssistedArmNavigationState::ConstPtr& msg) {

	ROS_INFO_ONCE("State callback received.");

	if (marker_pub_.getNumSubscribers() == 0) return;

	ROS_INFO_ONCE("We have some subscriber. Hooray!");

	visualization_msgs::MarkerArray arr;


	if (msg->joints_out_of_limits) {

			j_.header.stamp = ros::Time::now();

			arr.markers.push_back(j_);
			marker_pub_.publish(arr);
			return;
		}


	if (msg->planning_started) {

		m_.header.stamp = ros::Time::now();

		m_.text = "Position is fine";
		m_.color = color_def_;

		if (!msg->position_reachable) {

			m_.text = "Position can't be reached";
			m_.color = color_err_;

		}

		if (msg->position_in_collision) {

			m_.text = "Position is in collision";
			m_.color = color_err_;

		}

		arr.markers.push_back(m_);

	} // if planning started


	marker_pub_.publish(arr);




}


int main( int argc, char** argv ) {


  ros::init(argc, argv, "state_vis");

  StateVis sv;
  ros::spin();

}

