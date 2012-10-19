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

	  sdh_mode_client_ = nh_.serviceClient<cob_srvs::SetOperationMode>("set_mode_srv");

	  vel_publisher_ = nh_.advertise<brics_actuator::JointVelocities>("velocity_out", 10);

	  action_name_ = name;

	  time_to_stop_.resize(joints_.joints.size());

	  tact_sub_  = nh_.subscribe("tact_in", 10, &ReactiveGrasping::TactileDataCallback,this);
	  state_sub_ = nh_.subscribe("state_in", 10, &ReactiveGrasping::SdhStateCallback,this);
	  server_->start();

	  ROS_INFO("Action server started");

  }

}

void ReactiveGrasping::stop() {

	ROS_INFO("Immediately stopping.");

	std::vector<double> vel;
	vel.resize(joints_.joints.size());

	for (unsigned int i=0; i < vel.size(); i++)
		vel[i];

	publish(vel);

}

bool ReactiveGrasping::publish(std::vector<double> vel) {

	if (vel.size()!= joints_.joints.size()) {

		ROS_ERROR("Error on publishing velocities - vector size mismatch.");
		return false;

	}


	ros::Time now = ros::Time::now();

	brics_actuator::JointVelocities msg;

	for (unsigned int i=0; i < vel.size(); i++) {

		brics_actuator::JointValue jv;

		jv.timeStamp = now;
		jv.joint_uri = joints_.joints[i];
		jv.value = vel[i];

		msg.velocities.push_back(jv);

	}

	vel_publisher_.publish(msg);

	return true;

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

	  server_->setAborted(res,"Fatal error has occurred.");
	  return;

  }

  if (!setMode("velocity")) {

	  server_->setAborted(res,"Could not set SDH to velocity mode.");
	  return;

  }

  copyData();

  double t1 = goal->time.toSec() * (params_.a_ramp / 100.0);
  double t3 = goal->time.toSec() * (params_.d_ramp / 100.0);
  double t2 = goal->time.toSec() - (t1 + t3);

  ROS_INFO("t1=%f, t2=%f, t3=%f,tc=%f, rate=%f",t1,t2,t3,t1+t2+t3,params_.rate);
  ROS_INFO("Acc. will take %f %% of total time, dec. %f %%.",params_.a_ramp,params_.d_ramp);

  unsigned int num_of_acc = (unsigned int)ceil(t1 * params_.rate) - 1;
  unsigned int num_of_vel = (unsigned int)ceil(t2 * params_.rate);
  unsigned int num_of_dec = (unsigned int)ceil(t3 * params_.rate) - 1;

  // compute velocities for all non-static joints
  for (unsigned int i = 0; i < joints_.joints.size(); i++) {

	  if (joints_.is_static[i]) {

		  joints_.velocities[i].clear();
		  continue;

	  } else {


		  //double pos_diff = (goal->target_configuration.data[i] - sdh_data_act_.positions[i]);
		  double pos_diff = (sdh_data_act_.positions[i] - goal->target_configuration.data[i]);

		  double vel = (pos_diff / (0.5*t1 + t2 + 0.5*t3)); // constant velocity
		  double acc = (vel / t1) * (1.0 / params_.rate); // dv = a * dt = v/1 * 1/r
		  double dec = (vel / t3) * (1.0 / params_.rate);

		  if (vel>=params_.max_velocity) {

			  ROS_WARN("Joint %s will violate max. velocity limit (%f, limit %f).",joints_.joints[i].c_str(),vel,params_.max_velocity);

		  }

		  ROS_INFO("Joint %d, dv1=%f (%u), v2=%f (%u), dv3=%f (%u)",i,acc,num_of_acc,vel,num_of_vel,dec,num_of_dec);

		  joints_.velocities[i].resize(num_of_acc + num_of_vel + num_of_dec);

		  joints_.velocities[i][0] = 0.0;


		  for (unsigned int j=1; j < (num_of_acc + num_of_vel + num_of_dec); j++) {

			  if (j < num_of_acc) {

				  // acceleration
				  joints_.velocities[i][j] = joints_.velocities[i][j-1] + acc;

				  if (joints_.velocities[i][j] > vel) joints_.velocities[i][j] = vel;



			  } else if ( (j >= num_of_acc) && (j <= (num_of_acc + num_of_vel)) ) {

				  // constant velocity
				  joints_.velocities[i][j] = vel;

			  } else {

				  // deceleration
				  if ( (joints_.velocities[i][j-1] - dec) > 0) {

				  	joints_.velocities[i][j] = joints_.velocities[i][j-1] - dec;

				 } else {

					 joints_.velocities[i][j] = 0.0;

				 }

			  } // deceleration

			  //std::cout << joints_.velocities[i][j] << " ";

		  } // for num of vel

	  } // else

	  //std::cout << std::endl  << std::endl;

  }

  for(unsigned int i=0; i < joints_.joints.size(); i++) {

	  time_to_stop_[i] = -1.0;

  }

  ros::Time start_time = ros::Time::now();
  ros::Time max_time = ros::Time(0);

  if ( goal->time > ros::Duration(0) ) max_time = start_time + goal->time;
  else max_time = start_time + params_.default_time;

  unsigned int iter = 0;

  while(ros::ok()) {

	  // check for preempt request
	  if (server_->isPreemptRequested()) {

	         ros::Duration time_elapsed = ros::Time::now() - start_time;
	         double time_elapsed_d = time_elapsed.toSec();

	         ROS_INFO("%s: Preempted, elapsed time: %f", action_name_.c_str(),time_elapsed_d);

	         server_->setPreempted(res,std::string("preempted"));

	         stop();

	         setMode("position");

	         return;

	      }

	  // check for time condition
	  if ((ros::Time::now() >= max_time) || iter >= (num_of_acc + num_of_vel + num_of_dec) ) {

		  ros::Duration time_elapsed = ros::Time::now() - start_time;
		  double time_elapsed_d = time_elapsed.toSec();

		  ROS_INFO("Max. time condition reached (%fs, iter: %u)!",time_elapsed_d,iter);

		  res.actual_forces.data = tactile_data_;


		  res.actual_joint_values.data.resize(sdh_data_act_.positions.size());
		  for (unsigned int i=0; i < sdh_data_act_.positions.size(); i++) {

			  res.actual_joint_values.data[i] = (float)sdh_data_act_.positions[i];

		  }

		  ros::Duration d = ros::Time::now() - start_time;

		  for (unsigned int i=0; i < time_to_stop_.size(); i++) {

			  if (time_to_stop_[i] == -1.0) time_to_stop_[i] = d.toSec();

		  }

		  res.time_to_stop.data = time_to_stop_;

		  server_->setSucceeded(res,"Max. time condition reached.");

		  stop();

		  setMode("position");

		  return;

	  }

	  copyData();

	  // array for velocities
	  std::vector<double> vel;
	  vel.resize(joints_.joints.size());

	  // tdb
	  if (iter++ < (num_of_acc + num_of_vel + num_of_dec) ) {

		  // for each joint, assign correct velocity here
		  for(unsigned int i=0; i < joints_.joints.size(); i++) {

			  vel[i] = 0.0;

			  // skip static joints
			  if (joints_.is_static[i]) continue;

			  // skip joints which already reached their configuration
			  if (fabs(sdh_data_act_.positions[i] - goal->target_configuration.data[i]) < 0.02) {

				  if (time_to_stop_[i] == -1.0) {

					  ros::Duration d = ros::Time::now() - start_time;
					  time_to_stop_[i] = d.toSec();

					  ROS_INFO("Joint %s (%u) reached target configuration in %f secs.",joints_.joints[i].c_str(),i,time_to_stop_[i]);

				  }

				  continue;

			  }

			  // test force only for joints with tactile pad
			  if (joints_.has_tactile_pad[i]) {

				  if (tactile_data_[i] >= goal->max_force.data[i]) {

					  if (time_to_stop_[i] == -1.0) {

						  ros::Duration d = ros::Time::now() - start_time;
						  time_to_stop_[i] = d.toSec();

						  ROS_INFO("Joint %s (%u) reached max. force (%d) in %f secs.",joints_.joints[i].c_str(),i,tactile_data_[i],time_to_stop_[i]);

					  }

					  continue;

				  } else vel[i] = joints_.velocities[i][iter];


			  } else vel[i] = joints_.velocities[i][iter];



		  } // for over all joints

		  //iter++;

	  }

	  // publish velocities
	  publish(vel);

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

  if (fatal_error_) return;

  boost::mutex::scoped_lock(feedback_data_.data_mutex);

  feedback_data_.sdh_data = *msg;

  feedback_data_.sdh_data_stamp = ros::Time::now();

  if (!feedback_data_.sdh_data_checked) {

	  // check sizes of lists
	  if (feedback_data_.sdh_data.joint_names.size() != joints_.joints.size()) {

		  ROS_ERROR_ONCE("Received data for more/less joints than configured!");
		  fatal_error_ = true;
		  return;

	  }

	  // check names of joints and order
	  for (unsigned int i=0; i<joints_.joints.size(); i++) {

		  if (joints_.joints[i] != feedback_data_.sdh_data.joint_names[i]) {

			  ROS_ERROR_ONCE("Order or names of joint differs in config and received data!");
			  fatal_error_ = true;
			  return;

		  }

	  } // for

	  feedback_data_.sdh_data_checked = true;

  }

}


void ReactiveGrasping::TactileDataCallback(const schunk_sdh::TactileSensor::ConstPtr& msg) {

  if (fatal_error_) return;

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

		  if ( (msg->tactile_matrix[i].tactile_array[j]) > (max && msg->tactile_matrix[i].tactile_array[j] < 20000)) max = msg->tactile_matrix[i].tactile_array[j];

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
