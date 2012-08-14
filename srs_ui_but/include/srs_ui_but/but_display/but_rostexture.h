/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_ROS_IMAGE_TEXTURE_H
#define RVIZ_ROS_IMAGE_TEXTURE_H

#include <sensor_msgs/Image.h>

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreImage.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <tf/message_filter.h>

#include <stdexcept>

namespace tf
{
class TransformListener;
}


namespace srs_ui_but
{


	/**
	 * Connection between ROS and Ogre images. Internally uses OpenCV to convert between formats
	 */
	class CRosTextureConverter
	{
	public:
		/**
		 * String exception.
		 */
		class RTCException : public std::runtime_error
		{
		public:
			//! Constructor
			RTCException(const std::string& estr)
				  : std::runtime_error(estr)
		  {}

		}; // class RTCException

	public:
		//! Constructor
		CRosTextureConverter( const std::string & encoding = std::string(), bool bStaticTexture = false );

		//! Constructor - with texture name
		CRosTextureConverter( const std::string & name, const std::string & encoding, bool bStaticTexture = false );

		//! Destructor
		~CRosTextureConverter();

		//! Set output encoding
		void setEncoding( const std::string & encoding ){ m_output_encoding = encoding; }

		//! Set input image and convert it.
		bool convert( const sensor_msgs::Image::ConstPtr& image, const std::string & texture_name, bool writeInfo = false );

		//! Set input image and convert it. Internal name is used.
		bool convert( const sensor_msgs::Image::ConstPtr& image, bool writeInfo = false )
		{
			return convert( image, m_texture_name, writeInfo );
		}

		//! Get output ogre image
		Ogre::TexturePtr & getOgreTexture(){ return m_ogre_image; }

		//! Get output opencv image
		cv_bridge::CvImage * getOpenCVImage() { return m_cv_image.get(); }



	protected:
		//! Initialize Ogre texture
		void initOgreTexture( Ogre::PixelFormat format, const std::string & name );

		//! Convert string encoding to Ogre pixelformat
		Ogre::PixelFormat getFormat( const std::string & encoding );

		//! Write some texture stats
		void writeStats( Ogre::Image & image );

	protected:
		//! OpenCv image pointer
		cv_bridge::CvImagePtr m_cv_image;

		//! Ogre image pointer
		Ogre::TexturePtr m_ogre_image;

		//! Used output format
		std::string m_output_encoding;

		//! Used static texture pixel format
		Ogre::PixelFormat m_static_format;

		//! Used Ogre texture name
		std::string m_texture_name;

		//! Helper image
		Ogre::Image m_empty_image;

		//! Should be texture initialized only once?
		bool m_bStaticTexture;


	};  // class CRosTextureConverter

	class CRosTopicTexture
	{
	public:
		//! Constructor - encoding in OpenCV format
		CRosTopicTexture( const ros::NodeHandle& nh, const std::string & texture_name, const std::string & encoding );

		//! Constructor - encoding in Ogre format
		//CRosTopicTexture( const ros::NodeHandle& nh, const std::string & texture_name, Ogre::PixelFormat encoding );

		//! Set topic to subscribe to
		void setTopic(const std::string& topic);

		//! Get current topic
		std::string getTopic() { return m_topic; }

		//! Get current texture pointer
		const Ogre::TexturePtr& getTexture() { return m_texture_converter.getOgreTexture(); }

		//! Update content of texture from message
		bool update();

		//! Clear
		void clear();

	protected:
		//! On new image callback function
		void callback(const sensor_msgs::Image::ConstPtr& image);

	protected:
		//! Node handle
		ros::NodeHandle m_nh;

		//! Texture
		CRosTextureConverter m_texture_converter;

		//! Texture name
		std::string m_name;

		//! Current topic
		std::string m_topic;

		sensor_msgs::Image::ConstPtr m_current_image;
		image_transport::ImageTransport m_it;
		boost::shared_ptr<image_transport::SubscriberFilter> m_sub;
		boost::shared_ptr<tf::MessageFilter<sensor_msgs::Image> > m_tf_filter;
		std::string m_transport_type;
		boost::mutex m_mutex;
		bool m_new_image;
		std::string m_frame;
		tf::TransformListener* m_tf_client;

	};



} // namespace srs_ui_but


#endif
