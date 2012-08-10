/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 10.7.2012
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

#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/message_filter.h>

#define POINTCLOUD_TOPIC_NAME std::string("/cam3d/depth/points")
#define IMAGE_TOPIC_NAME	std::string("/srs_ui_but/depth/image")

namespace srs_ui_but
{

class CPCTransformer
{
protected:
	//! Incoming point cloud type
	typedef sensor_msgs::PointCloud2 tIncommingPointCloud;

	//! Published image type
	typedef sensor_msgs::Image tImage;

public:
	//! Constructor
	CPCTransformer()
	: m_pcFrameId("/head_cam3d_link")
	{
	}

	bool init()
	{
		ros::NodeHandle nh;

		// Try to get point cloud publisher name
		nh.param("pointcloud_topic_name", m_pcTopicName, POINTCLOUD_TOPIC_NAME );

		// Try to get publishing topic name
		nh.param("image_topic_name", m_imageTopicName, IMAGE_TOPIC_NAME );

		// Create publisher
		m_publisher = nh.advertise<tImage> (m_imageTopicName, 100, false);

		// Try to subscribe
		m_pcSubscriber  = new message_filters::Subscriber<tIncommingPointCloud>(nh, m_pcTopicName, 5);

		if (!m_pcSubscriber)
		{
			ROS_ERROR("Not subscribed...");
			//PERROR( "Not subscirbed to point clouds subscriber...");
			return false;
		}

	//	std::cerr << "Subscribed..." << std::endl;

		// Create message filter
		m_tfPointCloudSub = new tf::MessageFilter<tIncommingPointCloud>( *m_pcSubscriber, m_tfListener, m_pcFrameId, 5);
		m_tfPointCloudSub->registerCallback(boost::bind( &CPCTransformer::insertCloudCallback, this, _1));

		return true;
	}

protected:
	void insertCloudCallback(const tIncommingPointCloud::ConstPtr& cloud)
	{
		// No subscribers - no need to publish data...
		if( m_publisher.getNumSubscribers() == 0 )
			return;

	//	std::cerr << "Insert cloud callback. " << std::endl;

		sensor_msgs::ImagePtr ptrImage( new sensor_msgs::Image );

		// Set image parameters
		std::size_t  width = ptrImage->width = cloud->width;
		std::size_t  height = ptrImage->height = cloud->height;

		// No data?
		if( width * height == 0 )
		{
			std::cerr << "Not a dense cloud." << std::endl;
			return;
		}

		ptrImage->encoding = sensor_msgs::image_encodings::MONO16;
		ptrImage->step = ptrImage->width * sizeof( uint16_t );
		ptrImage->data.resize( height * ptrImage->step );

		// Get appropriate index
		int depth_index = -1;
		for (size_t d = 0; d < cloud->fields.size (); ++d)
		  if (cloud->fields[d].name == "z")
		  {
			  depth_index = (int) d;
			break;
		  }

		if( depth_index == -1 )
		{
			std::cerr << "No depth information found" << std::endl;
			return;
		}

//		std::cerr << "Image size: " << width << "x" << height << std::endl;

		// Get data
		int depth_offset = cloud->fields[depth_index].offset;
		int point_step = cloud->point_step;

		float min(1000000.0), max(-10000000.0 );

		for (size_t y = 0; y < height; y++)
		{
		  for (size_t x = 0; x < width; x++, depth_offset += point_step)
		  {
			uint16_t * pixel = reinterpret_cast<uint16_t *>(&(ptrImage->data[y * ptrImage->step + x * sizeof(uint16_t)]));

			float v(*reinterpret_cast<const float *>( &cloud->data[depth_offset] ));

			// Cut on kinect resolution distance (avg 6m)
//			if( v > 10.0 )
//				v = 10.0;

			if( v > max ) max = v;
			if( v < min ) min = v;

			*pixel =  uint16_t( v * 1000.0 );
		  }
		}

//		std::cerr << "Publishing. < " << min << ", " << max << " >" << std::endl;

		// all done, publish image
		m_publisher.publish( *ptrImage );

	}


protected:
	//! Point cloud topic name
	std::string m_pcTopicName;

	//! Point cloud subscriber name
	std::string m_imageTopicName;

	/// Subscriber - point cloud
	message_filters::Subscriber<tIncommingPointCloud> *m_pcSubscriber;

	//! Message filter (we only want point cloud 2 messages)
	tf::MessageFilter<tIncommingPointCloud> *m_tfPointCloudSub;

	/// Point cloud publisher
	ros::Publisher m_publisher;

	//! Transform listener
	tf::TransformListener m_tfListener;

	//! Used frame id (point cloud will be transformed to it)
	std::string m_pcFrameId;



}; // class CPCTransformer

} // namespace srs_ui_but

/**
 * Main function
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "but_depthtoimage_node");

	srs_ui_but::CPCTransformer transformer;

	if( !transformer.init() )
		return 0;

	ros::Rate loop_rate(10);

	while( ros::ok() )
	{
		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
} // main


