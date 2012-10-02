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

#include <srs_assisted_grasping/manual_grasping_node.h>

using namespace srs_assisted_grasping;
using namespace srs_assisted_grasping_msgs;
using namespace std;


ReactiveGrasping::ReactiveGrasping(std::string name) {


  // read all parameters
  double tmp;
  ros::param::param<double>("~default_time", tmp, 3.0);

  params_.default_time_ = ros::Duration(tmp);

  ros::param::param<double>("~max_force", params_.max_force_, 1000.0);
  ros::param::param<double>("~max_velocity", params_.max_velocity_, 1.5);
  ros::param::param<double>("~ramp", params_.ramp_, 10.0);
  ros::param::param<double>("~rate", params_.rate_, 5.0);


  ros::NodeHandle nh("~");
  XmlRpc::XmlRpcValue joints_list;


  if (nh.getParam("joints",joints_list)) {

	  std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;

	  for (i = joints_list.begin(); i != joints_list.end(); i++) {

		  joints_.joints.push_back(i->first);
		  joints_.has_tactile_pad.push_back((bool)i->second["has_tactile_pad"]);

	  }

	  ROS_INFO("Configured for %d joints with %d tactile pads",(int)joints_.joints.size(),(int)joints_.has_tactile_pad.size());


  } else {

	  ROS_ERROR("Can't get list of joints!");

  }





  server_ = new actionlib::SimpleActionServer<srs_assisted_grasping_msgs::ReactiveGraspingAction>(nh_, name, boost::bind(&ReactiveGrasping::execute, this, _1), false);

  sdh_mode_client_ = nh_.serviceClient<cob_srvs::SetOperationMode>("/sdh_controller/set_operation_mode");



  //vel_publisher_ = nh_.advertise<brics_actuator::JointVelocities>("/sdh_controller/command_vel", 10);

  action_name_ = name;

  tact_sub_  = nh_.subscribe("/sdh_controller/mean_values", 10, &ReactiveGrasping::TactileDataCallback,this);
  state_sub_ = nh_.subscribe("/sdh_controller/state", 10, &ReactiveGrasping::SdhStateCallback,this);
  server_->start();


}


void ReactiveGrasping::execute(const ReactiveGraspingGoalConstPtr &goal) {

  ROS_INFO("Reactive grasping action triggered");


  ReactiveGraspingResult res;
  ReactiveGraspingFeedback feedback;




}

void ReactiveGrasping::SdhStateCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr & msg) {

  data_mutex_.lock();

  data_mutex_.unlock();

}


void ReactiveGrasping::TactileDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {

  data_mutex_.lock();

  data_mutex_.unlock();

}


bool ReactiveGrasping::setMode(std::string mode) {


	cob_srvs::SetOperationMode srv;

	srv.request.operation_mode.data = mode;

	if (sdh_mode_client_.call(srv)) {

		if (srv.response.success.data) {

			ROS_INFO("SDH should be in %s mode",mode.c_str());
			return true;

		} else {

			ROS_ERROR("Failed to set SDH to %s mode, error: %s",mode.c_str(),srv.response.error_message.data.c_str());
			return false;

		}

	  } else {

		  ROS_ERROR("Failed to call SDH mode service");
		  return false;

	  }

}

int main(int argc, char** argv)
{


      ROS_INFO("Starting manual grasping node");
      ros::init(argc, argv, "but_reactive_grasping_node");

      ReactiveGrasping mg(string(ACT_GRASP));


      ROS_INFO("Spinning...");

      ros::spin();
      ros::shutdown();

}
