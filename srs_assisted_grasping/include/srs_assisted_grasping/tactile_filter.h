/******************************************************************************
 * \file tactile_filter.h
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
#ifndef TACTILE_FILTER_H
#define TACTILE_FILTER_H

#include <ros/ros.h>
#include <string>
#include <math.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>


#include "srs_assisted_grasping/services_list.h"
#include "srs_assisted_grasping/topics_list.h"


#include "schunk_sdh/TactileSensor.h"

#include <boost/thread.hpp>


namespace srs_assisted_grasping {

struct TactileFilterParams {

	bool median_prefilter;

	int median_width;

	int gaussian_width;

	int gaussian_height;

	double gaussian_sigma_x;

	double gaussian_sigma_y;

};

class TactileFilter {

	public:


		TactileFilter() {

			data_received_ = false;

			// read parameters
			ros::param::param<bool>("~median_prefilter",params_.median_prefilter,true);
			ros::param::param<int>("~median_width",params_.median_width,3);
			ros::param::param<int>("~gaussian_width",params_.gaussian_width,3);
			ros::param::param<int>("~gaussian_height",params_.gaussian_height,3);
			ros::param::param<double>("~gaussian_sigma_x",params_.gaussian_sigma_x,0.0);
			ros::param::param<double>("~gaussian_sigma_y",params_.gaussian_sigma_y,0.0);

			// check values of parameters
			if ((params_.median_width+1)%2!=0) {

				ROS_WARN("Param. 'median_width' must be odd and greater than zero! Using default value.");
				params_.median_width = 3;

			}

			if ( (params_.gaussian_height == 0) && (params_.gaussian_width == 0) ) {

				if (params_.gaussian_sigma_x==0) {

					ROS_WARN("sigma_x could not be zero while height and width are zero");
					params_.gaussian_sigma_x = 0.1; // ?????

				}


			} else {

				if ((params_.gaussian_width+1)%2!=0) {

					ROS_WARN("Param. 'gaussian_width' must be odd and greater than zero! Using default value.");
					params_.gaussian_width = 3;

				}

				if ((params_.gaussian_height+1)%2!=0) {

					ROS_WARN("Param. 'gaussian_height' must be odd and greater than zero! Using default value.");
					params_.gaussian_height = 3;

				}

			}

			if (params_.median_prefilter) {

				ROS_INFO("Using median prefilter, width: %d",params_.median_width);
			}

			ROS_INFO("Using gaussian filter with size(%d,%d), sigma(%f,%f)",params_.gaussian_width,params_.gaussian_height,params_.gaussian_sigma_x,params_.gaussian_sigma_y);

			tact_publisher_ = nh_.advertise<schunk_sdh::TactileSensor>("tact_out", 10);
			tact_subscriber_  = nh_.subscribe("tact_in", 10, &TactileFilter::TactileDataCallback,this);

			ROS_INFO("Initialized");

		}

		~TactileFilter() {



		}

	protected:

		ros::NodeHandle nh_;

		ros::Publisher tact_publisher_;
		ros::Subscriber tact_subscriber_;

		bool data_received_;

		void TactileDataCallback(const schunk_sdh::TactileSensor::ConstPtr & msg);

		boost::mutex tact_data_mutex_;
		std::vector<cv::Mat> tact_data_; // separate image for each pad
		std::vector<cv::Mat> tact_data_filtered_; // separate image for each pad

		//schunk_sdh::TactileSensor tact_data_;

		TactileFilterParams params_;

	private:



};

} // namespace

#endif /* TACTILE_FILTER_H */
