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


/*bool ManualGrasping::CloseRound(std::vector<double>& pos, std_msgs::Float32MultiArray& tact, uint8_t pos_t, uint8_t pos_f, uint8_t tact_t, uint8_t tact_f) {

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

		SetPhalanxPosition(pos,pos_f,0.9,inc_tmp);

	}

	if (tact.data[tact_t] < max_force_) {

		SetPhalanxPosition(pos,pos_t,0.0,inc_tmp);

	}

	return false;

}*/

/*bool ManualGrasping::CloseSquare(std::vector<double>& pos, std_msgs::Float32MultiArray& tact, uint8_t pos_t, uint8_t pos_f, uint8_t tact_t, uint8_t tact_f) {

	float inc_tmp = 0;

	if (tact.data[tact_f] < 0.1) inc_tmp = 3.0*inc_;
	else inc_tmp = inc_;

	if (tact.data[tact_f] > (max_force_/4.0) ) inc_tmp = inc_ / 2.0;


	if (tact.data[tact_f] < max_force_ ) {

		SetPhalanxPosition(pos,pos_t,0,inc_tmp);

		return false;

	} else return true;

}*/

/*bool ManualGrasping::SetPhalanxPosition(std::vector<double>& pos,uint8_t idx, double value, double inc) {


	if ( pos[idx] == value ) return true;


	if (pos[idx] >  value) {

		pos[idx] -= inc;

		if ( pos[idx] < value ) pos[idx] = value;

	} else {

		pos[idx] += inc;

		if ( pos[idx] > value ) pos[idx] = value;

		}

	return false;

}*/

/*bool ManualGrasping::InitializeFinger(std::vector<double>& pos,uint8_t pos_t, uint8_t pos_f) {

	const float inc_tmp = 5.0*inc_;

	const float def_t_pos = -0.9; //-0.85; //-0.9854;
	const float def_f_pos = 0.9; //0.7472;

	bool ready = true;

	if (!SetPhalanxPosition(pos,pos_t,def_t_pos,inc_tmp)) ready = false;
	if (!SetPhalanxPosition(pos,pos_f,def_f_pos,inc_tmp)) ready = false;

	return ready;


}*/


bool ManualGrasping::setPosition(std::vector<double> positions) {

	ROS_INFO("Sending position command to SDH controller");

	if (!setMode("position")) return false;

	trajectory_msgs::JointTrajectoryPoint jtp;

	jtp.positions = positions;
	jtp.velocities = velocities_;
	jtp.time_from_start = ros::Duration(0);


	std::vector<trajectory_msgs::JointTrajectoryPoint> vp;
	vp.push_back(jtp);

	control_msgs::FollowJointTrajectoryGoal sdh_goal;

	sdh_goal.trajectory.joint_names = joints_;
	sdh_goal.goal_time_tolerance = ros::Duration(10);

	sdh_goal.trajectory.joint_names = joints_;
	sdh_goal.trajectory.points = vp;

	sdh_trajectory_ctrl_client_->sendGoal(sdh_goal);

	if (!sdh_trajectory_ctrl_client_->waitForResult(ros::Duration(30.0))) {


		return false;

	} else {

		control_msgs::FollowJointTrajectoryResultConstPtr result = sdh_trajectory_ctrl_client_->getResult();

		if (result->error_code != 0) {

			ROS_WARN("SDH action - something is going wrong...");
			return false;

		} else return true;


	}

}

// opening of fingers using position interface
bool ManualGrasping::OpenGripper() {

	std::vector<double> positions_tmp;

	positions_tmp.push_back(0.0);
	positions_tmp.push_back(-0.9);
	positions_tmp.push_back(1.0);
	positions_tmp.push_back(-0.9);
	positions_tmp.push_back(1.0);
	positions_tmp.push_back(-0.9);
	positions_tmp.push_back(1.0);

	return setPosition(positions_tmp);


}


void ManualGrasping::PublishVelocities() {


	/*ros::Time now = ros::Time::now();

	for(unsigned int i=0;i<vel_.velocities.size();i++)
		vel_.velocities[i].timeStamp = now;

	vel_publisher_.publish(vel_);*/


	vel_publisher_.publish(vel_);

}

void ManualGrasping::stopMotion() {

	for(unsigned int i=0;i<joints_.size();i++) {
		//vel_.velocities[i].value = 0.0;
		vel_.data[i] = 0.0;
	  }

	  PublishVelocities();

	  // just for sure and to switch back to position mode
	  /*if (!setPosition(positions_)) {

	  	ROS_ERROR("Failed to fix final position of fingers.");

	  }*/

	  setMode("position");

}

void ManualGrasping::waitForNewData(double rate, std_msgs::Float32MultiArray& tactile_data, pr2_controllers_msgs::JointTrajectoryControllerState& sdh_data) {

  ros::Rate w(rate);


  bool data_received = false;

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


void ManualGrasping::execute(const ManualGraspingGoalConstPtr &goal) {

  ROS_INFO("Manual grasping action triggered");


  ManualGraspingResult res;
  ManualGraspingFeedback feedback;

  max_force_ = 1500;
  //inc_ = 0.0005;

  //inc_ = 0.01;

  if ((float)(goal->max_force) > max_force_) {

    ROS_INFO("Restricting grasping maximal force to %f (request was %f)",max_force_,(float)(goal->max_force));

  } else {

	  max_force_ = (float)(goal->max_force);

  }

  ros::Time start_time = ros::Time::now();

  ros::Time max_time = start_time + ros::Duration(45);


  // values from cob_default_robot_config sdh_joint_configurations.yaml
  // opened (script server) 0.0, -0.9854, 0.9472, -0.9854, 0.9472, -0.9854, 0.9472
  // closed (s.s.) [0.0, 0.0, 1.0472, 0.0, 1.0472, 0.0, 1.0472] -> asi spatne, controller tam neni schopen dojet
  // closed - spravne je asi same nuly.....
  // /sdh_controller/state -> pr2_controllers_msgs/JointTrajectoryControllerState -> desired, actual, error

  ros::Rate r(rate_);

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

	  waitForNewData(1.0,tactile_data,sdh_data);
	  data_received = true;

  }

  ROS_INFO("Waiting for sdh mode srv...");
  if (!ros::service::waitForService("/sdh_controller/set_operation_mode",ros::Duration(60))) {

	  ROS_ERROR("Service not available");

	  res.time_elapsed = ros::Time::now() - start_time;
	  server_->setAborted();

	  return;

  }

	// open gripper
	if (!initialized_) {


		if (!OpenGripper()) {

			ROS_WARN("SDH action timeouted, could not open fingers.");

			res.time_elapsed = ros::Time::now() - start_time;
			server_->setAborted();
			return;

		} else ROS_INFO("Gripper should be opened.");

		initialized_ = true;

	}


  //std::cout << "setting velocity mode" << std::endl;

  if (!setMode("velocity")) {

	res.time_elapsed = ros::Time::now() - start_time;
	server_->setAborted();
	return;


  }

  //std::cout << "Starting while loop" << std::endl;

  while(ros::ok()) {

	publish_command = false;

    if (ros::Time::now() > max_time) {

      stopMotion();

      res.grasped = false;
      res.time_elapsed = ros::Time::now() - start_time;
      server_->setAborted(res,"Timeout");

      ROS_ERROR("Timeout. Cannot reach desired positions");

      return;

    }

    if (server_->isPreemptRequested()) {

       stopMotion();

       ROS_INFO("%s: Preempted", action_name_.c_str());

       res.time_elapsed = ros::Time::now() - start_time;
       server_->setPreempted(res,string("preempted"));

       return;

    }

    //std::cout << "Trying to get data" << std::endl;

    data_mutex_.lock();
    tactile_data = tactile_data_;
    sdh_data = sdh_data_;
    data_mutex_.unlock();

    positions_ = sdh_data.actual.positions;
    velocities_ = sdh_data.actual.velocities;

    //std::cout << "Data ready, lets move it" << std::endl;


	/*feedback.tip1_force = tactile_data.data[0]; // fingertip 1
	feedback.tip2_force = tactile_data.data[2]; // 3
	feedback.tip3_force = tactile_data.data[4]; // 5*/

    feedback.tip1_force = tactile_data.data[0] > tactile_data.data[1] ? tactile_data.data[0] : tactile_data.data[1];
    feedback.tip2_force = tactile_data.data[2] > tactile_data.data[3] ? tactile_data.data[2] : tactile_data.data[3];
    feedback.tip3_force = tactile_data.data[4] > tactile_data.data[5] ? tactile_data.data[4] : tactile_data.data[5];

	server_->publishFeedback(feedback);


	//std::cout << "Feedback published" << std::endl;

	uint8_t fingers[3] = {0,0,0};

	if ( sdh_data.actual.positions[1] > -0.1 ) fingers[0] = 1;
	if ( sdh_data.actual.positions[3] > -0.1 ) fingers[1] = 1;
	if ( sdh_data.actual.positions[5] > -0.1 ) fingers[2] = 1;

	// check for zero position - is it really needed?
	if ( fingers[0]!=0 || fingers[1]!=0 || fingers[2]!=0) {

	  stopMotion();

	  ROS_ERROR("Manual grasping failed - one of the fingers reached 0 position.");
	  res.grasped = false;
	  res.time_elapsed = ros::Time::now() - start_time;
	  server_->setAborted(res,"grasping failed");

	  return;


	}


	for (uint8_t i=1; i < 4;i++)
		contact_last[i] = contact[i];

	//std::cout << "Grasp type: " << goal->grasp_type << std::endl;

	switch(goal->grasp_type) {

		// round
		case 0: {

			ROS_INFO_ONCE("Starting grasping, round type");

			for (unsigned int i=1;i<joints_.size()-1;i+=2) { // 6-1=4

				// no contact
				if ( (tactile_data.data[i-1]< min_contact_force_) && (tactile_data.data[i] < min_contact_force_) ) {

					ROS_INFO("No contact, idx %u",i);

					// acceleration ramp - for phalanx only
					if (velocities_[i]<max_speed_) {

						ROS_INFO("Starting acceleration ramp: idx %u",i);

						//vel_.velocities[i] = velocities_[i] + accel_;
						//vel_.data[i] = velocities_[i] + accel_;
						vel_.data[i] = vel_.data[i] + accel_;

						publish_command = true;

					} // ramp


					// deceleration
					if (positions_[i] > -0.2) {


						ROS_INFO("Starting deceleration ramp: idx %u, vel.: %f",i,velocities_[i]);

						float speed_coef = (0.2-fabs(positions_[i]))/0.2; // 0-1

						//vel_.velocities[i].value = speed_coef * vel_.velocities[i].value;
						vel_.data[i] = speed_coef * vel_.data[i];

						publish_command = true;

					}

				} else {

					ROS_INFO("Have some contact... (idx: %u, %u)",i,i+1);

					float bigger = tactile_data.data[i-1] > tactile_data.data[i] ? tactile_data.data[i-1] : tactile_data.data[i];

					if (bigger>=max_force_) {

						ROS_INFO_ONCE("Desired force (%f) reached (idx: %u, %u)",bigger,i-1,i);

						bigger = max_force_;

						contact[i] = true; // 1, 3, 5

					} else contact[i] = false;


					float speed_coef = (max_force_ - bigger)/max_force_; // 0-1

					// velocity should decrease to zero with increasing presure
					//vel_.velocities[i].value = max_speed_ * speed_coef;
					vel_.data[i] = max_speed_ * speed_coef;

					publish_command = true;


					} // if tactile data else


				} // for


			/*contact[1] = CloseRound(positions_,tactile_data,1,2,2,3); // thumb (positions: 1,2, tactile_data: 2,3)
			contact[2] = CloseRound(positions_,tactile_data,3,4,0,1); // finger 1
			contact[3] = CloseRound(positions_,tactile_data,5,6,4,5); // finger 2 -> weird one*/

		} break;

		// square
		case 1: {

			/*contact[1] = CloseSquare(positions_,tactile_data,1,2,2,3); // thumb (positions: 1,2, tactile_data: 2,3)
			contact[2] = CloseSquare(positions_,tactile_data,3,4,0,1); // finger 1
			contact[3] = CloseSquare(positions_,tactile_data,5,6,4,5); // finger 2 -> weird one*/

			stopMotion();

			ROS_ERROR("Not implemented type of grasping!");
			res.grasped = false;
			res.time_elapsed = ros::Time::now() - start_time;
			server_->setAborted(res,"grasping failed");

			return;

		} break;

		// cylindric
		case 2: {

			stopMotion();

			ROS_ERROR("Not implemented type of grasping!");
			res.grasped = false;
			res.time_elapsed = ros::Time::now() - start_time;
			server_->setAborted(res,"grasping failed");

			return;

		} break;

		default: {

			stopMotion();

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


    if (publish_command) {

    	PublishVelocities();


    } else {

    	/*ros::Duration w(0.5);
    	w.sleep();

    	waitForNewData(0.5,tactile_data,sdh_data);
    	positions_ = sdh_data.actual.positions;
    	velocities_ = sdh_data.actual.velocities;*/

    	stopMotion();

    	//setMode("position");
    	//stopMotion();

    	ROS_INFO("Manual grasping succeeded.");
    	res.grasped = true;
    	// send bigger one force for each finger as results
    	res.tip1_force = tactile_data.data[0] > tactile_data.data[1] ? tactile_data.data[0] : tactile_data.data[1];
    	res.tip2_force = tactile_data.data[2] > tactile_data.data[3] ? tactile_data.data[2] : tactile_data.data[3];
    	res.tip3_force = tactile_data.data[4] > tactile_data.data[5] ? tactile_data.data[4] : tactile_data.data[5];
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

bool ManualGrasping::setMode(std::string mode) {


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
