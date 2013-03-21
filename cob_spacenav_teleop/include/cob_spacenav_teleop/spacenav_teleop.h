/******************************************************************************
 * \file
 *
 * $Id: teleop_cob_marker.h 2037 2012-11-30 16:28:17Z spanel $
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

#ifndef TELEOPCOBSPACENAV_H_
#define TELEOPCOBSPACENAV_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <boost/signals2/mutex.hpp>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace cob_spacenav_teleop
{

struct Params {


	double max_vel_x;
	double max_vel_y;
	double max_vel_th;

	double scale_linear;
	double scale_angular;

	double sn_max_val;
    double sn_min_val_th;

    bool use_rviz_cam;
    std::string rviz_cam_link;
    std::string robot_base_link;

    double publish_rate;

    bool instant_stop_enabled;

    double ignore_th_high;
    double ignore_th_low;

    bool unsafe_limiter;

};

struct SpacenavData {

	geometry_msgs::Vector3 offset;
	geometry_msgs::Vector3 rot_offset;

	bool offset_received;
	bool rot_offset_received;

	boost::signals2::mutex mutex;

	ros::Time last_data;
	ros::Time last_nonzero_data;

};

struct Buttons {

	boost::signals2::mutex mutex;

	bool left;
	bool right;
	bool right_last;
	bool right_trigger;
};


class SpaceNavTeleop
{
public:

	SpaceNavTeleop();
	~SpaceNavTeleop();

	bool transf(std::string target_frame,geometry_msgs::PoseStamped& pose);

protected:

	ros::Subscriber offset_sub_;
	ros::Subscriber rot_offset_sub_;

	Params params_;

	void spacenavOffsetCallback(const geometry_msgs::Vector3ConstPtr& offset);
	void spacenavRotOffsetCallback(const geometry_msgs::Vector3ConstPtr& rot_offset);

	void timerCallback(const ros::TimerEvent& ev);
	ros::Timer timer_;

	void tfTimerCallback(const ros::TimerEvent& ev);
	ros::Timer tf_timer_;

	ros::Publisher twist_publisher_safe_;
	ros::Publisher twist_publisher_unsafe_;

	SpacenavData sn_data_;


	bool stop_detected_;
	ros::Time time_of_stop_;

	bool enabled_;
	ros::ServiceServer service_en_;
	ros::ServiceServer service_dis_;

	bool Enable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool Disable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	tf::TransformListener *tfl_;

	geometry_msgs::PoseStamped bp_;


	geometry_msgs::Vector3 GetAsEuler(geometry_msgs::Quaternion quat);
	void normAngle(double& a);

	//bool some_dir_limited_;
	bool x_pref_;
	bool y_pref_;
	bool z_pref_;

	ros::Time pref_time_;

	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	ros::Subscriber joy_sub_;
	bool publishing_to_unsafe_;
	bool robot_centric_mode_;

	Buttons btns_;

private:

};

}

#endif /* TELEOPCOBSPACENAV_H_ */
