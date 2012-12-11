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

#pragma once
#ifndef but_depthtexture_H_included
#define but_depthtexture_H_included

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <but_rostexture.h>
#include <cv_bridge/cv_bridge.h>
#include <OGRE/OgreMatrix4.h>
#include <image_geometry/pinhole_camera_model.h>

namespace srs_ui_but
{

namespace enc = sensor_msgs::image_encodings;

class CDepthImageConvertor
{
public:
	//! Consturctor
	CDepthImageConvertor(const ros::NodeHandle& nh, const std::string & topic);

	//! Destructor
	~CDepthImageConvertor();

	//! Get cv image access
	cv_bridge::CvImage * getBridgeImage() { return m_bridge_image.get(); }

	//! Get converted msg image
	sensor_msgs::ImagePtr & getSensorMsgsImage() { return m_depth_msg; }

	//! Get mutex lock
	boost::mutex & getMutexLock() { return m_mutex; }

	//! Has data?
	bool hasData() { return m_counter > 0; }

	//! Set topic to subscribe to
	void setTopic(const std::string& topic);

	//! Get current topic
	std::string getTopic() { return m_topic; }

	//! Set used projection matrix
	void setMatrix( const Ogre::Matrix4 & m ) { m_projectionMatrix = m; }

	//! Get camera model reference
	void  setCameraModel(const sensor_msgs::CameraInfo& msg);

protected:
	//! Callback function
	void depthCb(const sensor_msgs::ImageConstPtr& raw_msg);

	//! Convert depth image - just call appropriate method
	virtual bool convertDepth(const sensor_msgs::ImageConstPtr& raw_msg);

	//! Convert 16bit image to the internal representation
	virtual bool convertDepth16(const sensor_msgs::ImageConstPtr& raw_msg);

	//! COnvert 8bit image to the interna representation
	virtual bool convertDepth8(const sensor_msgs::ImageConstPtr& raw_msg);

	//! Convert image from depth to the visible version
	bool toVisible(int coord = 2);

	//! Convert image from depth to float (0->1) version
	bool toFloat( int coord = 2);

protected:
	// Subscriptions
	boost::shared_ptr<image_transport::ImageTransport> m_it;
	image_transport::Subscriber m_sub_raw;

	//! Publisher
	image_transport::Publisher m_pub_depth, m_pub_float;

	//! Converted message
	sensor_msgs::ImagePtr m_depth_msg, m_visible_msg, m_float_msg;

	//! Cv image
	cv_bridge::CvImagePtr m_bridge_image;

	//! Sizes range
	Ogre::Vector4 m_min_range, m_max_range;

	//! Internal images encoding
	std::string m_msg_encoding;
	std::string m_visible_encoding;

	//! Data locking mutex
	boost::mutex m_mutex;

	//! Image counter
	long m_counter;

	//! Subscribed topic name
	std::string m_topic;

	image_transport::TransportHints m_hints;

	//! Projection matrix
	Ogre::Matrix4 m_projectionMatrix;

	//! Camera model - must be set for conversion
	image_geometry::PinholeCameraModel m_camera_model;

	//! Was camera model initialized
	bool m_bCamModelInitialized;

}; // class CDepthImageConvertor


class CRosDepthTexture : public CDepthImageConvertor
{
public:
	//! Constructor
	CRosDepthTexture( const ros::NodeHandle& nh, const std::string & texture_name, const std::string& topic_name,  const std::string & encoding, bool bStaticTexture = false );

	//! Get current texture pointer
	const Ogre::TexturePtr& getTexture() { return m_texture_converter.getOgreTexture(); }


	//! Update content of texture from message
	//bool update();

	//! Clear
	void clear();

protected:
	//! Convert image to the internal representation
	virtual bool convertDepth(const sensor_msgs::ImageConstPtr& raw_msg);

protected:
	//! Used converter to ogre texture
	CRosTextureConverter m_texture_converter;



}; // class CRosDepthTexture

} // namespace srs_ui_but

//but_depthtexture_H_included
#endif

