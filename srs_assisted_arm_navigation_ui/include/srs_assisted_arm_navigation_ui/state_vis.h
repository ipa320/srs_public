/******************************************************************************
 * \file
 * \brief State visualization for srs_assisted_arm_navigation
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

#ifndef STATE_VIS_H_
#define STATE_VIS_H_


#include <ros/ros.h>
#include "srs_assisted_arm_navigation_msgs/AssistedArmNavigationState.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace srs_assisted_arm_navigation_ui {

class StateVis {

	public:

	  StateVis();
	  ~StateVis();


	protected:

	  void stateCallback(const srs_assisted_arm_navigation_msgs::AssistedArmNavigationState::ConstPtr& msg);

	  ros::Publisher marker_pub_;
	  ros::Subscriber arm_nav_state_sub_;

	  //std::vector<visualization_msgs::Marker> markers_;
	  visualization_msgs::Marker m_;
	  visualization_msgs::Marker j_;

	  std_msgs::ColorRGBA color_def_;
	  std_msgs::ColorRGBA color_err_;

	private:


};


}

#endif /* STATE_VIS_H_ */
