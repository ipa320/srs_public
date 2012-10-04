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

#include <srs_assisted_grasping/tactile_filter.h>

using namespace srs_assisted_grasping;

void TactileFilter::TactileDataCallback(const schunk_sdh::TactileSensor::ConstPtr & msg) {

	boost::mutex::scoped_lock(tact_data_mutex_);

	if (!data_received_) {

		tact_data_.resize(msg->tactile_matrix.size());
		tact_data_filtered_.resize(msg->tactile_matrix.size());

		data_received_ = true;

		ROS_INFO("First tactile data received (%u pads)",(unsigned int)tact_data_.size());

	}

	schunk_sdh::TactileSensor data = *msg;

	// convert it to opencv representation
	for (unsigned int i=0; i < data.tactile_matrix.size(); i++) {

		cv::Mat tmp;

		int x = data.tactile_matrix[i].cells_x;
		int y = data.tactile_matrix[i].cells_y;

		tmp = cv::Mat::zeros(x, y, CV_16S);

		for(int a = 0; a < x*y; a++) {

			tmp.at<int16_t>(a / y, a % x) = (int16_t)data.tactile_matrix[i].tactile_array[a];

		}

		tact_data_[i] = tmp;

	}


	// filter it
	for (unsigned int i=0; i < tact_data_.size(); i++) {

		tact_data_filtered_[i] = tact_data_[i].clone();

		cv::Scalar mean(0.0),stddev(0.0),mean_f(0.0),stddev_f(0.0);
		double min,min_f,max,max_f;

		min = min_f = max = max_f = 0;

		cv::meanStdDev(tact_data_[i],mean,stddev);

		if (params_.median_prefilter) {

			// prefilter with median - can be disabled by parameter
			cv::medianBlur(tact_data_filtered_[i],tact_data_filtered_[i],(unsigned int)params_.median_width);

		}

		cv::GaussianBlur(tact_data_filtered_[i],
				tact_data_filtered_[i],
				cv::Size((unsigned int)params_.gaussian_width, (unsigned int)params_.gaussian_height),
				(unsigned int)params_.gaussian_sigma_x,
				(unsigned int)params_.gaussian_sigma_y);

		cv::meanStdDev(tact_data_filtered_[i],mean_f,stddev_f);

		cv::minMaxLoc(tact_data_[i],&min,&max);
		cv::minMaxLoc(tact_data_filtered_[i],&min_f,&max_f);

		// print debugging info only if there are some values other than zero...
		if (mean.val[0]!=0) {

			//std::cout << "pad " << i << " " << mean.val[0] << ", filt: " << mean_f.val[0] << std::endl;

			ROS_DEBUG("Pad %u, unfiltered: %d / %d,  %d / %d, filtered: %d / %d,  %d / %d",i,
					(int16_t)mean.val[0], (int16_t)stddev.val[0], (int16_t)min, (int16_t)max,
					(int16_t)mean_f.val[0], (int16_t)stddev_f.val[0], (int16_t)min_f, (int16_t)max_f);

		}

	} // for


	// convert opencv representation back to TactileSensor message
	for (unsigned int i=0; i < data.tactile_matrix.size(); i++) {

			int x = data.tactile_matrix[i].cells_x;
			int y = data.tactile_matrix[i].cells_y;

			cv::Mat tmp = tact_data_[i];

			for(int a = 0; a < x*y; a++) {

				data.tactile_matrix[i].tactile_array[a] = tmp.at<int16_t>(a / y, a % x);

			}


		}

	// publish it
	tact_publisher_.publish(data);

}

int main(int argc, char** argv)
{

      ROS_INFO("Starting tactile filtering node");
      ros::init(argc, argv, "but_tactile_filtering_node");


      TactileFilter filter;

      ROS_INFO("Spinning...");

      ros::spin();
      ros::shutdown();

}
