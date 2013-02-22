/******************************************************************************
 * \file
 * \brief
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

#pragma once
#ifndef BUT_BBOVERLAP_NODE_H
#define BUT_BBOVERLAP_NODE_H

#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/InteractiveMarkerFeedback.h"
#include "visualization_msgs/Marker.h"
#include "srs_interaction_primitives/ScaleChanged.h"
#include <boost/thread.hpp>
#include "tf/transform_listener.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "boost/date_time/posix_time/posix_time.hpp"

namespace srs_user_tests {

typedef struct {

	geometry_msgs::Vector3 lwh;
	geometry_msgs::Pose pose;

} tbb;

typedef Eigen::Vector3f tpoint;
typedef std::vector<tpoint> tpoints;


class BBOverlap {

	public:

		BBOverlap();


		~BBOverlap();

		ros::Time last_log_out_;

		double points_volume(const tpoints &p);

		double rmin(double val1, double val2);
		double rmax(double val1, double val2);

	private:

	protected:

		bool publish_debug_markers_;

		double success_val_;

		ros::WallDuration success_min_dur_;
		ros::WallTime success_first_;
		ros::WallTime success_last_;

		ros::WallTime success_tmp_;

		ros::Subscriber sub_im_feedback_;
		ros::Subscriber sub_im_scale_;

		void im_feedback_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg);
		void im_scale_cb(const srs_interaction_primitives::ScaleChangedConstPtr& msg);

		void timer_cb(const ros::TimerEvent&);
		ros::Timer timer_;

		void update_points(tbb bb, tpoints &p);

		struct {

			boost::mutex mutex;
			visualization_msgs::Marker marker;
			//geometry_msgs::Vector3 lwh;
			tbb bb;
			ros::Publisher pub;
			double vol;
			tpoints points;

			ros::Publisher points_pub;

		} id_;

		struct {

			boost::mutex mutex;
			/*geometry_msgs::Vector3 lwh;
			geometry_msgs::Pose pose;*/
			tbb bb;
			tpoints points;

			ros::Publisher points_pub;

			bool scale_rec;
			bool pose_rec;

		} im_;

		ros::Publisher points_proc_pub_;

		void publish_points(tpoints points, ros::Publisher &pub, std_msgs::ColorRGBA c);

};

} // namespace

#endif
