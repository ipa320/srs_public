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

#include <srs_assisted_grasping/reactive_grasping_node.h>

using namespace srs_assisted_grasping;
using namespace srs_assisted_grasping_msgs;


bool ReactiveGrasping::readParams() {

  // read all parameters
  double tmp;
  ros::param::param<double>("~default_time", tmp, 3.0);

  params_.default_time = ros::Duration(tmp);

  ros::param::param<double>("~max_force", params_.max_force, 1000.0);
  ros::param::param<double>("~max_velocity", params_.max_velocity, 1.5);
  ros::param::param<double>("~a_ramp", params_.a_ramp, 10.0);
  ros::param::param<double>("~d_ramp", params_.d_ramp, 20.0);
  ros::param::param<double>("~rate", params_.rate, 20.0);

  // TODO add some checks for parameter values!!!


  ros::NodeHandle nh("~");
  XmlRpc::XmlRpcValue joints_list;

  joints_.num_of_tactile_pads = 0;


  if (nh.getParam("joints",joints_list)) {

	  ROS_ASSERT(joints_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	  for (int i=0; i < joints_list.size(); i++) {

		  XmlRpc::XmlRpcValue joint = joints_list[i];

		  if (joint.getType() != XmlRpc::XmlRpcValue::TypeStruct) {

			  ROS_ERROR("Wrong syntax in YAML config.");
			  return false;

		  }

		  if (!joint.hasMember("joint")) {

			  ROS_ERROR("Joint doesn't have 'joint' property defined.");
			  return false;

		  } else {

			  XmlRpc::XmlRpcValue joint_name = joint["joint"];

			  std::string tmp = static_cast<std::string>(joint_name);

			  joints_.joints.push_back(tmp);

			  std::cout << tmp << " ";

		  }

		  if (!joint.hasMember("static")) {

					  ROS_WARN("Joint doesn't have 'static' property defined. Supposing true.");

					  joints_.is_static.push_back(false);

				  } else {

			  XmlRpc::XmlRpcValue static_joint = joint["static"];

			  bool tmp = static_cast<bool>(static_joint);

			  joints_.is_static.push_back(tmp);

			  std::cout << "static: " << tmp << " ";
		  }

		  if (!joint.hasMember("has_tactile_pad")) {

			  ROS_WARN("Joint doesn't have 'has_tactile_pad' property defined. Supposing false.");

			  joints_.has_tactile_pad.push_back(false);

		  } else {

			  XmlRpc::XmlRpcValue has_pad = joint["has_tactile_pad"];

			  bool tmp = static_cast<bool>(has_pad);

			  joints_.has_tactile_pad.push_back(tmp);

			  std::cout << "has_pad: " << tmp << std::endl;

		  }

	  } // for


  } else {

	  ROS_ERROR("Can't get list of joints!");
	  return false;

  }

  for (unsigned char i=0; i < joints_.has_tactile_pad.size();i++) {

	 if (joints_.has_tactile_pad[i]) joints_.num_of_tactile_pads++;

  }

  return true;

}

ReactiveGrasping::~ReactiveGrasping(void) {

      server_->shutdown();
      delete server_;

    };



ReactiveGrasping::ReactiveGrasping(std::string name) {

  fatal_error_ = false;

  if (!readParams()) {

	  fatal_error_ = true;

  } else {

	  ROS_INFO("Configured for %d joints with %d tactile pads",(int)joints_.joints.size(),joints_.num_of_tactile_pads);

	  feedback_data_.tactile_data.resize(joints_.num_of_tactile_pads);
	  joints_.velocities.resize(joints_.joints.size());

	  feedback_data_.sdh_data_stamp = ros::Time(0);
	  feedback_data_.tactile_data_stamp = ros::Time(0);
	  feedback_data_.sdh_data_checked = false;

	  server_ = new actionlib::SimpleActionServer<srs_assisted_grasping_msgs::ReactiveGraspingAction>(nh_, name, boost::bind(&ReactiveGrasping::execute, this, _1), false);

	  sdh_mode_client_ = nh_.serviceClient<cob_srvs::SetOperationMode>("/sdh_controller/set_operation_mode");

	  vel_publisher_ = nh_.advertise<brics_actuator::JointVelocities>("velocity_out", 10);

	  action_name_ = name;

	  tact_sub_  = nh_.subscribe("tact_in", 10, &ReactiveGrasping::TactileDataCallback,this);
	  state_sub_ = nh_.subscribe("state_in", 10, &ReactiveGrasping::SdhStateCallback,this);
	  server_->start();

	  ROS_INFO("Action server started");

  }

}


void ReactiveGrasping::execute(const ReactiveGraspingGoalConstPtr &goal) {

  ROS_INFO("Reactive grasping action triggered");

  joints_.velocities.clear();

  ReactiveGraspingResult res;
  ReactiveGraspingFeedback feedback;

  ros::Rate r(params_.rate);

  // check if we have data available
  bool data_received = false;

  feedback_data_.data_mutex.lock();

  	  if (feedback_data_.sdh_data_stamp > ros::Time(0) &&
  		  feedback_data_.tactile_data_stamp > ros::Time(0)) data_received = true;

  feedback_data_.data_mutex.unlock();

  if (!data_received) {

	  ROS_ERROR("SDH/tactile data not available! Could not perform action...");

	  server_->setAborted(res,"no data received, could not perform action");
	  return;

  }

  if (fatal_error_) {

	  server_->setAborted(res,"fatal error has occurred");
	  return;

  }

  copyData();

  // compute velocities for all non-static joints
  for (unsigned int i = 0; i < joints_.joints.size(); i++) {

	  if (joints_.is_static[i]) {

		  joints_.velocities[i].clear();

	  } else {

		  // things to consider
		  // rate
		  // ramp - % of total time

		  double pos_diff = fabs(sdh_data_act_.positions[i] - goal->target_configuration.data[i]);

		  // acc -> const vel. -> dec
		  // velocity = s / t

		  unsigned int num_of_acc = (unsigned int)ceil(goal->time.toSec() * (params_.a_ramp / 100.0));
		  unsigned int num_of_dec = (unsigned int)ceil(goal->time.toSec() * (params_.d_ramp / 100.0));
		  unsigned int num_of_vel = (unsigned int)ceil(goal->time.toSec() * params_.rate) - (num_of_acc + num_of_dec);

		  joints_.velocities[i].resize(num_of_acc + num_of_vel + num_of_dec);

		  joints_.velocities[i][0] = 0.0;

		  // TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		  double acc = 0.0;
		  double dec = 0.0;


		  for (unsigned int j=1; j < num_of_vel; j++) {

			  if (j < num_of_acc) {

				  // acceleration
				  joints_.velocities[i][j] = joints_.velocities[i][j-1] + acc;



			  } else if ( (j >= num_of_acc) && (j <= (num_of_acc + num_of_vel)) ) {

				  // constant velocity
				  joints_.velocities[i][j] = joints_.velocities[i][j-1];

			  } else {

				  // deceleration
				  if ( (joints_.velocities[i][j-1] - dec) > 0) {

				  	joints_.velocities[i][j] = joints_.velocities[i][j-1] - dec;

				 } else {

					 joints_.velocities[i][j] = 0.0;

				 }

			  } // deceleration

		  } // for num of vel

	  } // else

  }

  ros::Time start_time = ros::Time::now();
  ros::Time max_time = ros::Time(0);

  if ( goal->time > ros::Duration(0) ) max_time = start_time + goal->time;
  else max_time = start_time + params_.default_time;


  while(ros::ok()) {

	  // check for preempt request
	  if (server_->isPreemptRequested()) {

	         ros::Duration time_elapsed = ros::Time::now() - start_time;

	         double time_elapsed_d = (double)time_elapsed.sec + (10e-9)*((double)time_elapsed.nsec);

	         ROS_INFO("%s: Preempted, elapsed time: %f", action_name_.c_str(),time_elapsed_d);

	         server_->setPreempted(res,std::string("preempted"));

	         return;

	      }

	  // check for time condition
	  if (ros::Time::now() >= max_time) {

		  ROS_INFO("Max. time condition reached!");

	  }

	  copyData();

	  //

	  r.sleep();

  } // while


}

void ReactiveGrasping::copyData() {

  // make copy of feedback data which is not accessed from callbacks
  feedback_data_.data_mutex.lock();

  sdh_data_act_ = feedback_data_.sdh_data.actual;
  tactile_data_ = feedback_data_.tactile_data;

  feedback_data_.data_mutex.unlock();

}

void ReactiveGrasping::SdhStateCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr & msg) {

  boost::mutex::scoped_lock(feedback_data_.data_mutex);

  feedback_data_.sdh_data = *msg;

  feedback_data_.sdh_data_stamp = ros::Time::now();

  if (!feedback_data_.sdh_data_checked) {

	  // check sizes of lists
	  if (feedback_data_.sdh_data.joint_names.size() != joints_.joints.size()) {

		  ROS_ERROR("Received data for more/less joints than configured!");
		  fatal_error_ = true;
		  return;

	  }

	  // check names of joints and order
	  for (unsigned int i=0; i<joints_.joints.size(); i++) {

		  if (joints_.joints[i] != feedback_data_.sdh_data.joint_names[i]) {

			  ROS_ERROR("Order or names of joint differs in config and received data!");
			  fatal_error_ = true;
			  return;

		  }

	  } // for

	  feedback_data_.sdh_data_checked = true;

  }

}


void ReactiveGrasping::TactileDataCallback(const schunk_sdh::TactileSensor::ConstPtr& msg) {

  boost::mutex::scoped_lock(feedback_data_.data_mutex);

  //feedback_data_.tactile_data = *msg;

  feedback_data_.tactile_data_stamp = ros::Time::now();

  if (! feedback_data_.tactile_data_checked) {

	  if (msg->tactile_matrix.size() != (unsigned int)joints_.num_of_tactile_pads) {

		ROS_ERROR("Received amount of tactile matrixes (%u) differs from configuration (%u)!",
				(unsigned int)joints_.num_of_tactile_pads,
				(unsigned int)feedback_data_.tactile_data.size());

		fatal_error_ = true;
		return;

	  }

	  feedback_data_.tactile_data_checked = true;
  }

  // for each pad find the maximum value
  for(unsigned int i=0; i < feedback_data_.tactile_data.size(); i++) {

	  int16_t max = 0;

	  for (unsigned int j=0; j < (unsigned int)(msg->tactile_matrix[i].cells_x*msg->tactile_matrix[i].cells_y); j++) {

		  if (msg->tactile_matrix[i].tactile_array[j] > max) max = msg->tactile_matrix[i].tactile_array[j] > max;

	  }

	  feedback_data_.tactile_data[i] = max;

  }

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


      ROS_INFO("Starting reactive grasping node");
      ros::init(argc, argv, "but_reactive_grasping_node");

      std::string tmp = ACT_GRASP;

      ReactiveGrasping mg(tmp);


      ROS_INFO("Spinning...");

      ros::spin();
      ros::shutdown();

}
