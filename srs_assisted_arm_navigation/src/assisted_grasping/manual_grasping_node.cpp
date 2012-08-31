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

#include <srs_assisted_arm_navigation/assisted_grasping/manual_grasping_node.h>

using namespace srs_assisted_arm_navigation;
using namespace std;


bool ManualGrasping::CloseRound(std::vector<double>& pos, std_msgs::Float32MultiArray& tact, uint8_t pos_t, uint8_t pos_f, uint8_t tact_t, uint8_t tact_f) {

	float bigger = tact.data[tact_f] > tact.data[tact_t] ? tact.data[tact_f] : tact.data[tact_t];

	float inc_tmp = 0;

	// speed-up closing when there is no contact and slow it down in presence of some contact with object
	if (bigger < 0.1) inc_tmp = 3.0*inc_;
	else inc_tmp = inc_;

	if (bigger > (max_force_/4.0) ) inc_tmp = inc_ / 2.0;

	// both parts of finger should reach max_force
	if ( (tact.data[tact_f] >= max_force_) && (tact.data[tact_t] >= max_force_) ) return true;


	// separate closing of both parts of finger
	if (tact.data[tact_f] < max_force_) {

		/*if (fabs(pos[pos_f])<inc_tmp) pos[pos_f] = 0;
		else pos[pos_f] += inc_tmp;*/

		SetPhalanxPosition(pos,pos_f,0.9,inc_tmp);

	}

	if (tact.data[tact_t] < max_force_) {

		/*if (fabs(pos[pos_t])<inc_tmp) pos[pos_t] = 0;
		else pos[pos_t] += inc_tmp;*/

		SetPhalanxPosition(pos,pos_t,0.0,inc_tmp);

	}

	return false;

}

bool ManualGrasping::CloseSquare(std::vector<double>& pos, std_msgs::Float32MultiArray& tact, uint8_t pos_t, uint8_t pos_f, uint8_t tact_t, uint8_t tact_f) {

	float inc_tmp = 0;

	if (tact.data[tact_f] < 0.1) inc_tmp = 3.0*inc_;
	else inc_tmp = inc_;

	if (tact.data[tact_f] > (max_force_/4.0) ) inc_tmp = inc_ / 2.0;


	if (tact.data[tact_f] < max_force_ ) {

		/*if (fabs(pos[pos_t])<inc_tmp) pos[pos_t] = 0;
		else pos[pos_t] += inc_tmp;*/

		SetPhalanxPosition(pos,pos_t,0,inc_tmp);

		return false;



	} else return true;

}

bool ManualGrasping::SetPhalanxPosition(std::vector<double>& pos,uint8_t idx, double value, double inc) {


	if ( pos[idx] == value ) return true;


	if (pos[idx] >  value) {

		pos[idx] -= inc;

		if ( pos[idx] < value ) pos[idx] = value;

	} else {

		pos[idx] += inc;

		if ( pos[idx] > value ) pos[idx] = value;

		}

	return false;

}

bool ManualGrasping::InitializeFinger(std::vector<double>& pos,uint8_t pos_t, uint8_t pos_f) {

	const float inc_tmp = 5.0*inc_;

	const float def_t_pos = -0.9; //-0.85; //-0.9854;
	const float def_f_pos = 0.9; //0.7472;

	bool ready = true;

	if (!SetPhalanxPosition(pos,pos_t,def_t_pos,inc_tmp)) ready = false;
	if (!SetPhalanxPosition(pos,pos_f,def_f_pos,inc_tmp)) ready = false;

	return ready;


}

void ManualGrasping::execute(const ManualGraspingGoalConstPtr &goal) {

  ROS_INFO("Manual grasping action triggered");

  trajectory_msgs::JointTrajectory jtr;
  ManualGraspingResult res;
  ManualGraspingFeedback feedback;

  max_force_ = 500;
  inc_ = 0.0005;

  if ((float)(goal->max_force) > max_force_) {

    ROS_INFO("Restricting grasping maximal force to %f (request was %f)",max_force_,(float)(goal->max_force));

  } else {

	  max_force_ = (float)(goal->max_force);

  }

  ros::Time start_time = ros::Time::now();

  ros::Time max_time = start_time + ros::Duration(30);

  jtr.header.stamp = ros::Time::now();
  jtr.joint_names = joints_;

  std::vector<double> positions;
  std::vector<double> velocities;

  // values from cob_default_robot_config sdh_joint_configurations.yaml
  // opened (script server) 0.0, -0.9854, 0.9472, -0.9854, 0.9472, -0.9854, 0.9472
  // closed (s.s.) [0.0, 0.0, 1.0472, 0.0, 1.0472, 0.0, 1.0472] -> asi spatne, controller tam neni schopen dojet
  // closed - spravne je asi same nuly.....
  // /sdh_controller/state -> pr2_controllers_msgs/JointTrajectoryControllerState -> desired, actual, error

  ros::Rate r(50);

  std_msgs::Float32MultiArray tactile_data;
  pr2_controllers_msgs::JointTrajectoryControllerState sdh_data;

  bool contact[4];
  bool contact_last[4];

  for (uint8_t i=0; i < 4;i++) {

	  contact[i] = false;
	  contact_last[i] = false;
  }

  if (goal->do_not_open_fingers) {

	  ROS_INFO("Let's start grasping without opening fingers.");
	  initialized_ = true;

  }  else {

	  ROS_INFO("First, try to put fingers into default position.");
	  initialized_ = false;

  }

  bool publish_command = false;

  bool data_received = false;

  // try to get data
  data_mutex_.lock();
  if (sdh_received_ && tact_received_) {

	  data_received = true;
	  tactile_data = tactile_data_;
	  sdh_data = sdh_data_;

  }
  data_mutex_.unlock();


  if (!data_received) {

	  ros::Rate w(1);

	  while (!data_received && ros::ok()) {

		  w.sleep();

		  ROS_INFO("Waiting for data...");

		  data_mutex_.lock();

		  if (sdh_received_ && tact_received_) {

			  data_received = true;
			  tactile_data = tactile_data_;
			  sdh_data = sdh_data_;

		  }

		  data_mutex_.unlock();

	  }


  }


  for(uint8_t i=0; i < joints_.size(); i++) {

	  positions_[i] = sdh_data_.actual.positions[i];

  }

  while(ros::ok()) {

	publish_command = false;

    if (ros::Time::now() > max_time) {

      res.grasped = false;
      res.time_elapsed = ros::Time::now() - start_time;
      server_->setAborted(res,"Timeout");

      ROS_ERROR("Timeout. Cannot reach desired positions");
      break;

    }

    if (server_->isPreemptRequested()) {

       ROS_INFO("%s: Preempted", action_name_.c_str());

       res.time_elapsed = ros::Time::now() - start_time;
       server_->setPreempted(res,string("preempted"));

       return;

    }

    data_mutex_.lock();
    tactile_data = tactile_data_;
    sdh_data = sdh_data_;
    data_mutex_.unlock();

	// open gripper
	if (!initialized_) {

		if (!InitializeFinger(positions_,1,2)) publish_command = true;
		if (!InitializeFinger(positions_,3,4)) publish_command = true;
		if (!InitializeFinger(positions_,5,6)) publish_command = true;

		if (!publish_command) {

			ROS_INFO("Fingers are opened, let's start grasping.");
			initialized_ = true;

		}

	}

	if (initialized_) {

		feedback.tip1_force = tactile_data.data[1];
		feedback.tip2_force = tactile_data.data[3];
		feedback.tip3_force = tactile_data.data[5];

		server_->publishFeedback(feedback);


		uint8_t fingers[3] = {0,0,0};

		if ( (ceil(sdh_data.actual.positions[1]*10) == 0.0) ) fingers[0] = 1;
		if ( (ceil(sdh_data.actual.positions[3]*10) == 0.0) ) fingers[1] = 1;
		if ( (ceil(sdh_data.actual.positions[5]*10) == 0.0) ) fingers[2] = 1;

		// is this really needed????
		if ( fingers[0]!=0 || fingers[1]!=0 || fingers[2]!=0) {

		  ROS_INFO("Manual grasping failed - one of the fingers reached 0 position.");
		  res.grasped = false;
		  res.time_elapsed = ros::Time::now() - start_time;
		  server_->setAborted(res,"grasping failed");

		  return;


		}


		for (uint8_t i=1; i < 4;i++)
			contact_last[i] = contact[i];

		switch(goal->grasp_type) {

			// round
			case 0: {

				contact[1] = CloseRound(positions_,tactile_data,1,2,2,3); // thumb (positions: 1,2, tactile_data: 2,3)
				contact[2] = CloseRound(positions_,tactile_data,3,4,0,1); // finger 1
				contact[3] = CloseRound(positions_,tactile_data,5,6,4,5); // finger 2 -> weird one

			} break;

			// square
			case 1: {

				contact[1] = CloseSquare(positions_,tactile_data,1,2,2,3); // thumb (positions: 1,2, tactile_data: 2,3)
				contact[2] = CloseSquare(positions_,tactile_data,3,4,0,1); // finger 1
				contact[3] = CloseSquare(positions_,tactile_data,5,6,4,5); // finger 2 -> weird one

			} break;

			// cylindric
			case 2: {

				ROS_ERROR("Not implemented type of grasping!");
				res.grasped = false;
				res.time_elapsed = ros::Time::now() - start_time;
				server_->setAborted(res,"grasping failed");

				return;

			} break;

			default: {

				ROS_ERROR("Not implemented type of grasping!");
				res.grasped = false;
				res.time_elapsed = ros::Time::now() - start_time;
				server_->setAborted(res,"grasping failed");

				return;

			} break;

		} // switch

		if (contact[1] && (!contact_last[1]) ) ROS_INFO("Finger 1 now has contact.");
		if (contact[2] && (!contact_last[2]) ) ROS_INFO("Finger 2 now has contact.");
		if (contact[3] && (!contact_last[3]) ) ROS_INFO("Finger 3 now has contact.");

		if (contact_last[1] && (!contact[1]) ) ROS_INFO("Finger 1 now has no contact.");
		if (contact_last[2] && (!contact[2]) ) ROS_INFO("Finger 2 now has no contact.");
		if (contact_last[3] && (!contact[3]) ) ROS_INFO("Finger 3 now has no contact.");

		if (goal->accept_two_fingers_contact) {

			if (contact[1] && (contact[2] || contact[3])) publish_command = false;
			else publish_command = true;

		} else {

			if (contact[1] && contact[2] && contact[3]) publish_command = false;
			else publish_command = true;

		}

	} // init.

    if (publish_command) {

    	trajectory_msgs::JointTrajectoryPoint jtp;
    	jtp.positions = positions_;
    	jtp.velocities = velocities_;
    	jtp.time_from_start = ros::Duration(0);
    	jtr.points.push_back(jtp);
    	jt_publisher_.publish(jtr);

    } else {

    	ROS_INFO("Manual grasping succeeded.");
    	res.grasped = true;
    	res.tip1_force = tactile_data.data[1]; // finger
    	res.tip2_force = tactile_data.data[3]; // thumb
    	res.tip3_force = tactile_data.data[5]; // finger
    	res.time_elapsed = ros::Time::now() - start_time;
    	server_->setSucceeded(res,"grasping should be fine");

    	return;

    };

    r.sleep();

  } // while




}

void ManualGrasping::SdhStateCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr & msg) {

  data_mutex_.lock();
  sdh_received_ = true;
  sdh_data_.actual = msg->actual;
  sdh_data_.desired = msg->desired;
  sdh_data_.error = msg->error;
  sdh_data_.header = msg->header;
  //sdh_data_.joint_names = msg->joint_names;
  data_mutex_.unlock();

}


void ManualGrasping::TactileDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {

  data_mutex_.lock();
  tact_received_=true;
  tactile_data_.data = msg->data;
  tactile_data_stamp_ = ros::Time::now();
  data_mutex_.unlock();

}

void ManualGrasping::addJoint(std::string joint) {

  joints_.push_back(joint);

  positions_.push_back(0.0);
  velocities_.push_back(0.0);

}

void ManualGrasping::inited(bool val) {

  inited_ = val;

}

int main(int argc, char** argv)
{


      ROS_INFO("Starting manual grasping node");
      ros::init(argc, argv, "but_manual_grasping_node");

      //ros::NodeHandle n;

      string tmp(ACT_GRASP);

      ManualGrasping mg(tmp);

      mg.addJoint("sdh_knuckle_joint");
      mg.addJoint("sdh_thumb_2_joint");
      mg.addJoint("sdh_thumb_3_joint");
      mg.addJoint("sdh_finger_12_joint");
      mg.addJoint("sdh_finger_13_joint");
      mg.addJoint("sdh_finger_22_joint");
      mg.addJoint("sdh_finger_23_joint");

      mg.inited(true);

      ROS_INFO("Spinning...");

      ros::spin();
      ros::shutdown();

}
