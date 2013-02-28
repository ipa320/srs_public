/******************************************************************************
 * \file
 *
 * $Id: cob_interactive_teleop.cpp 674 2012-04-19 13:59:19Z spanel $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Zdenek Materna (imaterna@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 1/12/2012
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

#include "cob_spacenav_teleop/spacenav_teleop.h"

using namespace cob_spacenav_teleop;

SpaceNavTeleop::SpaceNavTeleop() {

	enabled_ = true;

	tfl_ = new tf::TransformListener();

	ros::param::param<double>("~max_vel_x",params_.max_vel_x,0.3);
	ros::param::param<double>("~max_vel_y",params_.max_vel_y,0.2);
	ros::param::param<double>("~max_vel_th",params_.max_vel_th,0.3);
	ros::param::param<double>("~scale_linear",params_.scale_linear,0.5);
	ros::param::param<double>("~scale_angular",params_.scale_angular,0.25);

	ros::param::param<double>("~spacenav/max_val",params_.sn_max_val,350.0);
	ros::param::param<double>("~spacenav/min_val_th",params_.sn_min_val_th,0.05);

	ros::param::param<double>("~spacenav/ignore_th_high",params_.ignore_th_high,0.9);
	ros::param::param<double>("~spacenav/ignore_th_low",params_.ignore_th_low,0.1);

	ros::param::param<bool>("~spacenav/instant_stop_enabled",params_.instant_stop_enabled,false);

	ros::param::param<bool>("~unsafe_limiter",params_.unsafe_limiter,false);

	ros::param::param<bool>("~use_rviz_cam",params_.use_rviz_cam,false);
	ros::param::param<std::string>("~rviz_cam_link",params_.rviz_cam_link,"/rviz_cam");

	params_.robot_base_link = "/base_link";

	if (params_.sn_min_val_th > 0.5) params_.sn_min_val_th = 0.5;
	if (params_.sn_min_val_th < 0.0) params_.sn_min_val_th = 0.0;

	// make sure that all values are positive
	if (params_.sn_max_val < 0.0) params_.sn_max_val *= -1.0;
	if (params_.max_vel_x < 0.0) params_.max_vel_x *= -1;
	if (params_.max_vel_y < 0.0) params_.max_vel_y *= -1;
	if (params_.max_vel_th < 0.0) params_.max_vel_th *= -1;

	// and limit maximal values

  btns_.left = false;
  btns_.right = false;
  btns_.right_last = false;
  btns_.right_trigger = false;

	stop_detected_ = false;
	//some_dir_limited_ = false;
	x_pref_ = false;
	y_pref_ = false;
	z_pref_ = false;

	pref_time_ = ros::Time(0);

	bp_.header.frame_id = params_.rviz_cam_link;
	bp_.header.stamp = ros::Time(0);
	bp_.pose.position.x = 0;
	bp_.pose.position.y = 0;
	bp_.pose.position.z = 0;
	bp_.pose.orientation.x = 0.0;
	bp_.pose.orientation.y = 0.0;
	bp_.pose.orientation.z = 0.0;
	bp_.pose.orientation.w = 1.0;

	sn_data_.last_data = ros::Time(0);
	sn_data_.last_nonzero_data = ros::Time(0);

	ros::NodeHandle nh("~");

	offset_sub_ = nh.subscribe("/spacenav/offset",3,&SpaceNavTeleop::spacenavOffsetCallback,this);
	rot_offset_sub_ = nh.subscribe("/spacenav/rot_offset",3,&SpaceNavTeleop::spacenavRotOffsetCallback,this);
	joy_sub_ = nh.subscribe<sensor_msgs::Joy>("/spacenav/joy",3,&SpaceNavTeleop::joyCallback,this);

	twist_publisher_safe_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_safe",3);
	twist_publisher_unsafe_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_unsafe",3);

	publishing_to_unsafe_ = false;
	robot_centric_mode_ = false;

	// 50 Hz
	timer_ = nh.createTimer(ros::Duration(0.02),&SpaceNavTeleop::timerCallback,this);

	service_en_ = nh.advertiseService("enable", &SpaceNavTeleop::Enable,this);
	service_dis_ = nh.advertiseService("disable", &SpaceNavTeleop::Disable,this);

	if (params_.instant_stop_enabled) ROS_INFO("Instant stop feature enabled.");
	else ROS_INFO("Instant stop feature disabled.");

	if (params_.unsafe_limiter) ROS_INFO("Unsafe limiter activated.");
	else ROS_INFO("Unsafe limiter NOT activated.");

	ROS_INFO("Initiated...");


}

SpaceNavTeleop::~SpaceNavTeleop() {

	delete tfl_;


}

void SpaceNavTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	if ((int)(joy->buttons.size()) != 2) return;

	btns_.mutex.lock();

	btns_.right_last = btns_.right;

	btns_.left = joy->buttons[0];
	btns_.right = joy->buttons[1];

	// activate "triger" only if teleop is enabled
	if (enabled_ && (!btns_.right_last) && btns_.right) btns_.right_trigger = true;

	btns_.mutex.unlock();

}

bool SpaceNavTeleop::Enable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

	ROS_INFO("Spacenav teleop enabled.");
	enabled_ = true;
	return true;

}

bool SpaceNavTeleop::Disable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

	ROS_INFO("Spacenav teleop disabled.");
	enabled_ = false;
	return true;

}

void SpaceNavTeleop::normAngle(double& a) {

	if (a > 2*M_PI) {

		a -= 2*M_PI;
		return;

	}

	if (a < 0) {

		a = 2*M_PI + a;

	}

	return;

}

// Return the rotation in Euler angles
geometry_msgs::Vector3 SpaceNavTeleop::GetAsEuler(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 vec;

  btQuaternion q;

  btScalar roll, pitch, yaw;

  tf::quaternionMsgToTF(quat, q);
  btMatrix3x3(q).getRPY(roll,pitch,yaw);

  vec.x = (double)roll;
  vec.y = (double)pitch;
  vec.z = (double)yaw;

  normAngle(vec.x);
  normAngle(vec.y);
  normAngle(vec.z);

  return vec;

}


void SpaceNavTeleop::timerCallback(const ros::TimerEvent& ev) {

	ROS_INFO_ONCE("Timer triggered.");

	if (sn_data_.offset_received && sn_data_.rot_offset_received) ROS_INFO_ONCE("COB SpaceNav Teleop is running.");
	else return;

	if ( params_.use_rviz_cam && (!tfl_->frameExists(params_.rviz_cam_link)) ) {

		ROS_ERROR_ONCE("We are going to use RVIZ camera position but it's TF frame (%s) doesn't exist! This message will apper only once.",params_.rviz_cam_link.c_str());
		return;

	}

	if (!enabled_) return;

	if (twist_publisher_safe_.getNumSubscribers() == 0) {

		ROS_WARN_ONCE("We have no subscriber on main (safe) topic.");

		return;

	} else {

		ROS_INFO_ONCE("Someone is subscribed. Starting publishing...");

	}

	geometry_msgs::Twist tw;

	tw.linear.x = 0.0;
	tw.linear.y = 0.0;
	tw.linear.z = 0.0;

	tw.angular.x = 0.0;
	tw.angular.y = 0.0;
	tw.angular.z = 0.0;

	geometry_msgs::Vector3 offset;
	geometry_msgs::Vector3 rot_offset;

	ros::Time last_data;

	sn_data_.mutex.lock();

	offset = sn_data_.offset;
	rot_offset = sn_data_.rot_offset;
	last_data = sn_data_.last_data;

	sn_data_.mutex.unlock();

	// instant STOP
	if ( ((ros::Time::now() - last_data) > ros::Duration(0.5)) && (last_data != ros::Time(0)) ) {

		// spacenav node probably crashed ?
		ROS_WARN("Old data");
		twist_publisher_safe_.publish(tw);

		return;

	}


	if (!stop_detected_) {

		if ( (offset.z < -0.85) && params_.instant_stop_enabled) {

			stop_detected_ = true;
			ROS_WARN("Instant stop!");
			time_of_stop_ = ros::Time::now();

			twist_publisher_safe_.publish(tw);
			return;

		}


	} else {

		if ((ros::Time::now() - time_of_stop_) > ros::Duration(5.0)) {

			stop_detected_ = false;
			ROS_INFO("Releasing instant stop.");

		} else {

			twist_publisher_safe_.publish(tw);
			return;

		}

	}

	if ( (ros::Time::now() - sn_data_.last_nonzero_data ) > ros::Duration(1.0)) {

		// we have nothing to publish
		return;

	}

	// allow different ways of control
	if (fabs(rot_offset.y) > fabs(offset.x)) offset.x = rot_offset.y;
	if (fabs(rot_offset.x) > fabs(offset.y)) offset.y = -rot_offset.x;


	// filter out too small values
	if (fabs(offset.x) < params_.sn_min_val_th ) offset.x = 0;
	if (fabs(offset.y) < params_.sn_min_val_th ) offset.y = 0;
	if (fabs(rot_offset.z) < params_.sn_min_val_th ) rot_offset.z = 0;

	bool unsafe = false;
	bool mode_trigger = false;

	btns_.mutex.lock();
	unsafe = btns_.left;
	mode_trigger = btns_.right_trigger;
	btns_.right_trigger = false;

	btns_.mutex.unlock();

	if (!params_.use_rviz_cam) {

		ROS_INFO_ONCE("Started in mode without using RVIZ camera position.");
		robot_centric_mode_ = true;

	} else {

		if (robot_centric_mode_) {

			if (mode_trigger) {

				ROS_INFO("Switching back to user centric mode.");
				robot_centric_mode_ = false;

			}

		} else {

			if (mode_trigger) {

				robot_centric_mode_ = true;
				ROS_INFO("Switching to robot centric mode.");

			}

		}

	}

	if (!robot_centric_mode_) {

	// transformation of velocities vector is not needed for turning in place
	//if (!rot) {

		ROS_INFO_ONCE("Started in mode with using RVIZ camera position.");

		// this is necessary only if we don't want to rotate

		geometry_msgs::PoseStamped bp = bp_;

		if (!transf(params_.robot_base_link,bp)) return;

		geometry_msgs::Vector3 rpy = GetAsEuler(bp.pose.orientation);

		// this is angle between robot base_link and RVIZ camera
		ROS_DEBUG("YAW: %f",(rpy.z/(2*M_PI))*360);

		Eigen::Vector3f vec((float)offset.x,(float)offset.y,(float)offset.z);

		Eigen::AngleAxisf tr((float)rpy.z, Eigen::Vector3f::UnitZ());

		// we are rotating velocities vector around Z axis
		vec = tr * vec;

		offset.x = (double)vec(0);
		offset.y = (double)vec(1);
		offset.z = (double)vec(2);

	}


	// will we prefer some direction?
	if ( (fabs(offset.x) > params_.ignore_th_high) && (fabs(offset.y) < params_.ignore_th_low) && (fabs(rot_offset.z) < params_.ignore_th_low)) {

		if (!x_pref_) {

			x_pref_ = true;
			y_pref_ = false;
			z_pref_ = false;
			pref_time_ = ros::Time::now();
			ROS_INFO("Preferring x dir");

		}

	} else

	if ( (fabs(offset.y) > params_.ignore_th_high) && (fabs(offset.x) < params_.ignore_th_low) && (fabs(rot_offset.z) < params_.ignore_th_low) ) {

		if (!y_pref_) {

			x_pref_ = false;
			y_pref_ = true;
			z_pref_ = false;
			pref_time_ = ros::Time::now();
			ROS_INFO("Preferring y dir");

		}

	} else

	if ( (fabs(offset.z) > params_.ignore_th_high) && (fabs(offset.x) < params_.ignore_th_low) && (fabs(rot_offset.y) < params_.ignore_th_low) ) {

		if (!z_pref_) {

			x_pref_ = false;
			y_pref_ = false;
			z_pref_ = true;
			pref_time_ = ros::Time::now();
			ROS_INFO("Preferring z rot");

		}

	} else {


		if (x_pref_ || y_pref_ || z_pref_) {

			if ( (ros::Time::now() - pref_time_) > ros::Duration(0.5) ) {

				x_pref_ = false;
				y_pref_ = false;
				z_pref_ = false;

				ROS_INFO("No dir preferred.");

			}

		}

	}

	if (x_pref_) {

		offset.y = 0.0;
		rot_offset.z = 0.0;

	}


	if (y_pref_) {

		offset.x = 0.0;
		rot_offset.z = 0.0;

	}

	if (z_pref_) {

		offset.x = 0.0;
		rot_offset.y = 0.0;

	}


	// scale to velocities for COB
	offset.x *= params_.max_vel_x;
	offset.y *= params_.max_vel_y;

	rot_offset.z *= params_.max_vel_th;


	tw.linear = offset;
	tw.angular = rot_offset;


	if (unsafe) {

		if (!publishing_to_unsafe_) {

			publishing_to_unsafe_ = true;
			ROS_WARN("Publishing to unsafe topic!");

		}

		if (params_.unsafe_limiter) {

			tw.angular.z = 0.0;
			tw.linear.x = 0.0;
			tw.linear.y /= 3.0;

		}

		twist_publisher_unsafe_.publish(tw);

	} else {

		if (publishing_to_unsafe_) {

			ROS_INFO("Publishing to safe topic again.");
			publishing_to_unsafe_ = false;

		}

		twist_publisher_safe_.publish(tw);

	}


	return;

}

bool SpaceNavTeleop::transf(std::string target_frame,geometry_msgs::PoseStamped& pose) {

	// transform pose of camera into world
	try {

			if (tfl_->waitForTransform(target_frame, pose.header.frame_id, pose.header.stamp, ros::Duration(2.0))) {

			  tfl_->transformPose(target_frame,pose,pose);

			} else {

			  ROS_ERROR("Could not get TF transf. from %s into %s.",pose.header.frame_id.c_str(),target_frame.c_str());
			  return false;

			}

	} catch(tf::TransformException& ex){
	   std::cerr << "Transform error: " << ex.what() << std::endl;

	   return false;
	}

	return true;

}

void SpaceNavTeleop::spacenavOffsetCallback(const geometry_msgs::Vector3ConstPtr& offset) {

	sn_data_.mutex.lock();

	sn_data_.offset = *offset;

	// limit max. value
	if (sn_data_.offset.x > params_.sn_max_val) sn_data_.offset.x = params_.sn_max_val;
	if (sn_data_.offset.x < -params_.sn_max_val) sn_data_.offset.x = -params_.sn_max_val;

	if (sn_data_.offset.y > params_.sn_max_val) sn_data_.offset.y = params_.sn_max_val;
	if (sn_data_.offset.y < -params_.sn_max_val) sn_data_.offset.y = -params_.sn_max_val;

	if (sn_data_.offset.z > params_.sn_max_val) sn_data_.offset.z = params_.sn_max_val;
	if (sn_data_.offset.z < -params_.sn_max_val) sn_data_.offset.z = -params_.sn_max_val;


	// normalize
	sn_data_.offset.x /= params_.sn_max_val;
	sn_data_.offset.y /= params_.sn_max_val;
	sn_data_.offset.z /= params_.sn_max_val;

	sn_data_.last_data = ros::Time::now();

	if (sn_data_.offset.x != 0.0 || sn_data_.offset.y != 0.0) sn_data_.last_nonzero_data = ros::Time::now();

	sn_data_.mutex.unlock();

	if (!sn_data_.offset_received) {

		sn_data_.offset_received = true;
		ROS_INFO("Offset received.");

	}

}


void SpaceNavTeleop::spacenavRotOffsetCallback(const geometry_msgs::Vector3ConstPtr& rot_offset) {

	sn_data_.mutex.lock();

	sn_data_.rot_offset = *rot_offset;

	if (sn_data_.rot_offset.x > params_.sn_max_val) sn_data_.rot_offset.x = params_.sn_max_val;
	if (sn_data_.rot_offset.x < -params_.sn_max_val) sn_data_.rot_offset.x = -params_.sn_max_val;


	if (sn_data_.rot_offset.y > params_.sn_max_val) sn_data_.rot_offset.y = params_.sn_max_val;
	if (sn_data_.rot_offset.y < -params_.sn_max_val) sn_data_.rot_offset.y = -params_.sn_max_val;

	if (sn_data_.rot_offset.z > params_.sn_max_val) sn_data_.rot_offset.z = params_.sn_max_val;
	if (sn_data_.rot_offset.z < -params_.sn_max_val) sn_data_.rot_offset.z = -params_.sn_max_val;

	sn_data_.rot_offset.x /= params_.sn_max_val;
	sn_data_.rot_offset.y /= params_.sn_max_val;
	sn_data_.rot_offset.z /= params_.sn_max_val;

	if (sn_data_.rot_offset.z != 0.0) sn_data_.last_nonzero_data = ros::Time::now();

	sn_data_.mutex.unlock();

	if (!sn_data_.rot_offset_received) {

		sn_data_.rot_offset_received = true;
		ROS_INFO("Rot. offset received.");

	}

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cob_spacenav_teleop");

  ROS_INFO("Starting COB SpaceNav Teleop...");
  SpaceNavTeleop sp;

  /*ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();*/

  ros::spin();

}
