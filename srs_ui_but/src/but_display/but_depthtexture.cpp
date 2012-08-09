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
 * Date: dd/mm/2011
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

#include "but_depthtexture.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <limits>

#define PUBLISHING_TOPIC "/srs_ui_but/depth/normalized_image"
#define FLOAT_IMAGE_TOPIC "/srs_ui_but/depth/float_image"



//=======================================================================================================================
//		CDepthImageConvertor
//

/**
 * Constructor
 */
srs_ui_but::CDepthImageConvertor::CDepthImageConvertor(const ros::NodeHandle& nh, const std::string & topic)
: m_depth_msg( new sensor_msgs::Image)
, m_visible_msg( new sensor_msgs::Image )
, m_float_msg( new sensor_msgs::Image )
, m_msg_encoding(enc::TYPE_32FC4)
, m_visible_encoding( enc::MONO8 )
, m_topic( topic )
, m_hints("raw", ros::TransportHints(), nh)
, m_bCamModelInitialized( false )
{
	m_it.reset(new image_transport::ImageTransport(nh));


	m_sub_raw = m_it->subscribe(topic, 1, &CDepthImageConvertor::depthCb, this, m_hints);

	m_pub_depth = m_it->advertise(PUBLISHING_TOPIC, 1 );
	m_pub_float = m_it->advertise(FLOAT_IMAGE_TOPIC, 1 );

	// Allocate new Image message

	std::cerr << "... CDepthImageConvertor: depth texture initialized..." << std::endl;
}

/**
 * Callback function
 */
void srs_ui_but::CDepthImageConvertor::depthCb(const sensor_msgs::ImageConstPtr& raw_msg)
{
	boost::mutex::scoped_lock lock(m_mutex);

	// Convert image
	convertDepth(raw_msg);
	toVisible(2);
	toFloat(2);
}


/**
 *  Convert image to the internal representation
 */
bool srs_ui_but::CDepthImageConvertor::convertDepth(const sensor_msgs::ImageConstPtr& raw_msg)
{
	if( ! m_bCamModelInitialized )
		return false;

	if (raw_msg->encoding != enc::MONO16)
	{
		std::cerr << "... CDepthImageConvertor: Expected data of type " << enc::TYPE_16UC1.c_str() << ", got " << raw_msg->encoding.c_str() << std::endl;
		ROS_ERROR("Expected data of type [%s], got [%s]", enc::TYPE_16UC1.c_str(),raw_msg->encoding.c_str());
		return false;
	}

	typedef uint16_t tIPixel;
	typedef float tOPixel;

	m_depth_msg->header   = raw_msg->header;
	m_depth_msg->encoding = m_msg_encoding;
	m_depth_msg->height   = raw_msg->height;
	m_depth_msg->width    = raw_msg->width;
	m_depth_msg->step     = raw_msg->width * 4 * sizeof (tOPixel);
	m_depth_msg->data.resize( m_depth_msg->height * m_depth_msg->step);

	float bad_point = std::numeric_limits<float>::quiet_NaN ();

	// Compute image centers
	float center_x( m_camera_model.cx() ), center_y( m_camera_model.cy() );

//	std::cerr << m_camera_model.fx() << ", " << m_camera_model.fy() << std::endl;

	// Scaling
	float constant_x( 0.001 / m_camera_model.fx() ), constant_y( 0.001 / m_camera_model.fy() );

	// Fill in the depth image data, converting mm to m
	const tIPixel* raw_data = reinterpret_cast<const tIPixel*>(&raw_msg->data[0]);
	tOPixel* depth_data = reinterpret_cast<tOPixel*>(&m_depth_msg->data[0]);

	// Clear ranges
	for( int i = 0; i < 4; ++i )
	{
		m_max_range[i] = -1000000000.0;
		m_min_range[i] =  1000000000.0;
	}

	Ogre::Vector4 v;
	v[3] = 0.0;

	float x( 0.0 ), y( 0.0 ), depth;
//	long counter(0);

	for (unsigned index = 0; index < m_depth_msg->height * m_depth_msg->width; ++index )
	{
		tIPixel raw = raw_data[index ];

		// Distance cut-off
		if( raw > 70000 )
			raw = 70000;

		if( raw == 0 )
		{
			v[0] = v[1] = v[2] = v[3] = bad_point;
		}
		else
		{
			depth = float(raw);

			v[0] = (x - center_x) * depth * constant_x;
			v[1] = (y - center_y) * depth * constant_y;
			v[2] = depth * 0.001;
			v[3] = sqrt( v[0] * v[0] + v[1] * v[1] + v[2] * v[2] );
		}


//		v = v * m_projectionMatrix;
/*
		if( v[3] == 0.0 )
			++counter;
		else
			v /= v[3];
//*/

		depth_data[4*index] = v[0]; //(tOPixel)value;
		depth_data[4*index + 1] = v[1]; //(tOPixel)value;
		depth_data[4*index + 2] = v[2]; //(tOPixel)value;
		depth_data[4*index + 3] = v[3];

		for( int i = 0; i < 4; ++i )
		{
			if( v[i] > m_max_range[i] )
				m_max_range[i] = v[i];

			if( v[i] < m_min_range[i] )
				m_min_range[i] = v[i];
		}

		++x;
		if( x > raw_msg->width )
		{
			x -= raw_msg->width;
			++y;
		}
	}

//	std::cerr << "All: " << m_depth_msg->height * m_depth_msg->width << ", zero: " << zr << std::endl;
/*
	std::cerr << "Ranges: " << std::endl;
	for( int i = 0; i < 4; ++i )
	{
		std::cerr << "< " << m_min_range[i] << " - " << m_max_range[i] << " >"<< std::endl;
	}
*/
//	std::cerr << "Bad points: " << counter << std::endl;

	++m_counter;

	return true;
}

/**
 * Convert depth image to the visible format
 */
bool srs_ui_but::CDepthImageConvertor::toVisible( int coord )
{
	typedef uint8_t tVPixel;

	m_visible_msg->header = m_depth_msg->header;
	m_visible_msg->encoding = m_visible_encoding;
	m_visible_msg->height = m_depth_msg->height;
	m_visible_msg->width = m_depth_msg->width;
	m_visible_msg->step = m_depth_msg->width * sizeof(tVPixel);
	m_visible_msg->data.resize(m_visible_msg->height * m_visible_msg->step);

	tVPixel c;

	float divider(1.0);

	if (m_max_range[coord] != m_min_range[coord])
		divider = 1.0 / (m_max_range[coord] - m_min_range[coord]);

	for (unsigned int y = 0; y < m_depth_msg->height; ++y) {
		float *f = reinterpret_cast<float *> (&m_depth_msg->data[y
				* m_depth_msg->step]);
		tVPixel *v = reinterpret_cast<tVPixel*> (&m_visible_msg->data[y
				* m_visible_msg->step]);

		for (unsigned int x = 0; x < m_depth_msg->width; ++x) {
			c = (tVPixel) (255 * ((f[4 * x + coord] - m_min_range[coord]) * divider));
			v[x] = c;
			//			v[3*x+1] = c;
			//			v[3*x+2] = c;


		}
	}
	/*
	 for (unsigned index = 0; index < m_visible_msg->height * m_visible_msg->width; ++index)
	 {
	 c = (uint8_t) (255*((depth_data[index]-m_min_range)*divider));

	 visible_data[oindex] = c; ++oindex;
	 visible_data[oindex] = c; ++oindex;
	 visible_data[oindex] = c; ++oindex;
	 }
	 */
	// Publish
	m_pub_depth.publish(m_visible_msg);

	return true;
}

/**
 * Convert depth image to the visible format
 */
bool srs_ui_but::CDepthImageConvertor::toFloat( int coord )
{
	typedef uint8_t tVPixel;

	m_float_msg->header = m_depth_msg->header;
	m_float_msg->encoding = m_visible_encoding;
	m_float_msg->height = m_depth_msg->height;
	m_float_msg->width = m_depth_msg->width;
	m_float_msg->step = m_depth_msg->width * sizeof(tVPixel);
	m_float_msg->data.resize(m_float_msg->height * m_float_msg->step);

	tVPixel c;

	float depth;

	for (unsigned int y = 0; y < m_depth_msg->height; ++y) {
		float *f = reinterpret_cast<float *> (&m_depth_msg->data[y * m_depth_msg->step]);
		tVPixel *v = reinterpret_cast<tVPixel*> (&m_float_msg->data[y * m_float_msg->step]);

		for (unsigned int x = 0; x < m_depth_msg->width; ++x)
		{
			depth = f[4 * x + coord] * 0.5 + 0.5;

			c = (depth < 0.0 ) ? 0 : (depth > 1.0 ) ? 255 : depth * 255;
			v[x] = c;
			//			v[3*x+1] = c;
			//			v[3*x+2] = c;


		}
	}

	// Publish
	m_pub_float.publish(m_float_msg);

	return true;
}

/**
 *  Set topic to subscribe to
 */
void srs_ui_but::CDepthImageConvertor::setTopic(const std::string& topic)
{
	boost::mutex::scoped_lock lock(m_mutex);

	if( topic.empty() )
		m_sub_raw.shutdown();
	else
	{
		m_sub_raw = m_it->subscribe(topic, 1, &CDepthImageConvertor::depthCb, this, m_hints);
	}

	m_topic = topic;
}

/**
 *  Get camera model reference
 */
void  srs_ui_but::CDepthImageConvertor::setCameraModel(const sensor_msgs::CameraInfo& msg)
{
	m_bCamModelInitialized = m_camera_model.fromCameraInfo( msg );

//	if( m_bCamModelInitialized )
//		std::cerr << "CDepthImageConvertor::setCameraModel - initialized." << std::endl;
}

//=================================================================================================
//		CRosDepthTexture

srs_ui_but::CRosDepthTexture::CRosDepthTexture( const ros::NodeHandle& nh, const std::string & texture_name, const std::string& topic_name, const std::string & encoding, bool bStaticTexture /*= false*/ )
: CDepthImageConvertor( nh, topic_name )
, m_texture_converter( texture_name, encoding, true )
{

}

/**
 * Update content of texture from message
 */
bool srs_ui_but::CRosDepthTexture::convertDepth(const sensor_msgs::ImageConstPtr& raw_msg)
{
	CDepthImageConvertor::convertDepth( raw_msg );

//	std::cerr << "...Depth: Converting image. M_COUNTER: " << m_counter << std::endl;

	if( hasData() )
	{
//		std::cerr << "...Depth: Converting image..." << std::endl;
		return m_texture_converter.convert( m_depth_msg, false );
	}

	return false;
}

/**
 * Clear
 */
void srs_ui_but::CRosDepthTexture::clear()
{

}

