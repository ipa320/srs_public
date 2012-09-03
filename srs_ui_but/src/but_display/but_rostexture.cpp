/*
 * but_rostexture.cpp
 *
 *  Created on: 7.6.2012
 *      Author: wik
 */

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

#include "but_rostexture.h"
#include "sensor_msgs/image_encodings.h"

#include <tf/tf.h>

#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreColourValue.h>

/*******************************************************************************************
 *  CRosTextureConverter
 *******************************************************************************************/

/**
 * Consturctor
 */
srs_ui_but::CRosTextureConverter::CRosTextureConverter( const std::string & encoding, bool bStaticTexture /*= false*/ )
: m_output_encoding( encoding )
, m_bStaticTexture( bStaticTexture )
{
	if( bStaticTexture )
	{
		// Get pixel format
		try
		{
			m_static_format = getFormat(encoding);
		}
		catch (RTCException & e)
		{
			std::cerr << "CRosTextureConverter::CRosTextureConverter exception: " << e.what() << std::endl;
			ROS_ERROR( "CRosTextureConverter::CRosTextureConverter exception: %s", e.what() );
			return;
		}

		// Initialize static texture
		initOgreTexture( m_static_format, "RosTexture ");
	}

//	std::cerr << "CRosTextureConverter::CRosTextureConverter done" << std::endl;

}

/**
 * Constructor - with texture name
 */
srs_ui_but::CRosTextureConverter::CRosTextureConverter( const std::string & name, const std::string & encoding, bool bStaticTexture /*= false*/ )
: m_output_encoding( encoding )
, m_texture_name( name )
, m_bStaticTexture( bStaticTexture )
{
	if( bStaticTexture )
	{

		// Get pixel format
		try
		{
			m_static_format = getFormat(encoding);
		}
		catch (RTCException & e)
		{
			std::cerr << "CRosTextureConverter::CRosTextureConverter exception: " << e.what() << std::endl;
			ROS_ERROR( "CRosTextureConverter::CRosTextureConverter exception: %s", e.what() );
			return;
		}


		if( m_texture_name.size() == 0)
		{
			std::cerr << "CRosTextureConverter::CRosTextureConverter exception: Wrong texture name - zero length. " << std::endl;
			throw RTCException( "Wrong texture name - zero length." );
			return;
		}

		// Initialize static texture
		initOgreTexture( m_static_format, name );
	}

//	std::cerr << "CRosTextureConverter::CRosTextureConverter done" << std::endl;

}

/**
 * Destructor
 */
srs_ui_but::CRosTextureConverter::~CRosTextureConverter( )
{

}

/**
 * Set input image and convert it.
 */
bool srs_ui_but::CRosTextureConverter::convert( const sensor_msgs::Image::ConstPtr& image, const std::string & texture_name, bool writeInfo )
{
	if( writeInfo)
		std::cerr << "CRosTextureConverter::convert. IE: " << image->encoding << " OE: " << m_output_encoding << std::endl;

	try{
		// Take input and convert it to the opencv image...
		m_cv_image = cv_bridge::toCvCopy( image, m_output_encoding );
	}
	catch (cv_bridge::Exception& e)
	{
		std::cerr << "cv_bridge exception: " << e.what() << std::endl;
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}

	Ogre::PixelFormat format( Ogre::PF_UNKNOWN );

	if( !m_bStaticTexture )
	{
		std::string encoding( m_output_encoding );

		// Output encoding not defined, so use input one
		if( encoding.length() == 0 )
		{
			encoding = m_cv_image->encoding;
		}

		// Initialize ogre texture
		try{
			format = getFormat( encoding );
			initOgreTexture( format, texture_name );
		}
		catch( RTCException & e)
		{
			std::cerr << "Texture converter exception: " << e.what() << std::endl;
			ROS_ERROR( "Texture converter exception: %s", e.what() );
			return false;
		}
	}else{

		format = m_static_format;
	}

	// Convert texture

	// Get data access and size
	uchar *data_ptr = m_cv_image->image.ptr();
	cv::Size size = m_cv_image->image.size();
	size_t data_size( size.area() * m_cv_image->image.elemSize() );
	int width = size.width;
	int height = size.height;

	// Buffer image
	Ogre::Image ogre_image;

	// Create pixel stream
	Ogre::DataStreamPtr pixel_stream;
	pixel_stream.bind(new Ogre::MemoryDataStream(data_ptr, data_size));

	try
	{
		// Load data
		ogre_image.loadRawData(pixel_stream, width, height, 1, format, 1, 0);

//	if( m_bFlip )
//		ogre_image.flipAroundX();
	}
	catch (Ogre::Exception& e)
	{
		// TODO: signal error better
		ROS_ERROR("Error loading image: %s", e.what());
		return false;
	}

	m_ogre_image->unload();
	m_ogre_image->loadImage(ogre_image);

	if( writeInfo )
		writeStats( ogre_image );

	return true;
}

/**
 * Convert string encoding to Ogre pixelformat
 */
Ogre::PixelFormat srs_ui_but::CRosTextureConverter::getFormat( const std::string & encoding)
{
	if( encoding == "CV_8UC1" || encoding == "mono8" )
		return Ogre::PF_BYTE_L;

	if( encoding == "CV_8UC2" )
		return Ogre::PF_BYTE_LA;

	if( encoding == "bgr8" || encoding == "CV_8UC3" || encoding == "bgr8: CV_8UC3" || encoding == "CV_8SC3" || encoding == "bgr8: CV_8SC3" )
		return Ogre::PF_BYTE_BGR;

	if( encoding == "rgb8" || encoding == "rgb8: CV_8UC3" || encoding == "rgb8: CV_8SC3")
		return Ogre::PF_BYTE_RGB;

	if( encoding == "rgba8" || encoding == "CV_8UC4" || encoding == "rgba8: CV_8UC4" || encoding == "CV_8SC4" || encoding == "rgba8: CV_8SC4")
		return Ogre::PF_BYTE_RGBA;

	if( encoding == "bgra8" || encoding == "bgra8: CV_8UC4" || encoding == "bgra8: CV_8SC4")
		return Ogre::PF_BYTE_BGRA;

	if( encoding == "CV_32FC1" || encoding == "32FC1" )
		return Ogre::PF_FLOAT32_R;

	if( encoding == "CV_32FC2" || encoding == "32FC2")
		return Ogre::PF_FLOAT32_GR;

	if( encoding == "CV_32FC3" || encoding == "32FC3" )
		return Ogre::PF_FLOAT32_RGB;

	if( encoding == "CV_32FC4" || encoding == "32FC4")
		return Ogre::PF_FLOAT32_RGBA;

	if( encoding == "mono16" )
	{
		return Ogre::PF_FLOAT16_R;
	}

	throw RTCException( "Unknown input format: " + encoding );

	return Ogre::PF_UNKNOWN;
}

/**
 * Initialize Ogre texture
 */
void srs_ui_but::CRosTextureConverter::initOgreTexture( Ogre::PixelFormat format, const std::string & name )
{
	if( m_ogre_image.get() != 0 && m_ogre_image->getFormat() == format )
	{
		throw RTCException( "Cannot initialize ogre texture: " + name );
		return;
	}

	m_empty_image.load("no_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

	m_ogre_image = Ogre::TextureManager::getSingleton().createManual(name,
			  Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	 			Ogre::TEX_TYPE_2D, // Type
	 			m_empty_image.getWidth(),	// Width
	 			m_empty_image.getWidth(),	// Height
	 			1, 		// Depth
	 			0,		// Number of mipmaps
	 			format,	// Pixel format
	 			Ogre::TU_DYNAMIC_WRITE_ONLY); // Usage
}

/**
 * Write some texture stats
 */
void srs_ui_but::CRosTextureConverter::writeStats( Ogre::Image & image )
{
	size_t width( image.getWidth() ), height( image.getHeight() ), x, y;

	Ogre::ColourValue c;

	double rmin( 100000), gmin( 100000), bmin( 100000), amin( 100000);
	double rmax(-100000), gmax(-100000), bmax(-100000), amax(-100000);

	for( y = 0; y < height; ++y )
	{
		for( x = 0; x < width; ++x )
		{
			c = image.getColourAt( x, y, 0 );

			if( rmin > c.r ) rmin = c.r;
			if( gmin > c.g ) gmin = c.g;
			if( bmin > c.b ) bmin = c.b;
			if( amin > c.a ) amin = c.a;

			if( rmax < c.r ) rmax = c.r;
			if( gmax < c.g ) gmax = c.g;
			if( bmax < c.b ) bmax = c.b;
			if( amax < c.a ) amax = c.a;

		}
	}

	std::cerr << "Image size:     " << width << " x " << height << std::endl;
	std::cerr << "Encoding: 	  " << m_output_encoding << std::endl;
	std::cerr << "R: < " << rmin << ", " << rmax << " >" << std::endl;
	std::cerr << "G: < " << gmin << ", " << gmax << " >" << std::endl;
	std::cerr << "B: < " << bmin << ", " << bmax << " >" << std::endl;
	std::cerr << "A: < " << amin << ", " << amax << " >" << std::endl;
}

/****************************************************************************************************
 * CRosTopicTexture
 ****************************************************************************************************/


/**
 * Constructor
 */
srs_ui_but::CRosTopicTexture::CRosTopicTexture( const ros::NodeHandle& nh, const std::string & texture_name, const std::string & encoding )
: m_nh( nh )
, m_texture_converter( texture_name, encoding, true )	// Create static texture converter
, m_name( texture_name )
, m_it(nh)
, m_transport_type("raw")
, m_new_image(false)
, m_tf_client(0)
{
	std::cerr << "CRosTopicTexture::CRosTopicTexture" << std::endl;
}

/**
 * Set topic to subscribe to
 */

void srs_ui_but::CRosTopicTexture::setTopic(const std::string& topic)
{
	boost::mutex::scoped_lock lock(m_mutex);
	  // Must reset the current image here because image_transport will unload the plugin as soon as we unsubscribe,
	  // which removes the code segment necessary for the shared_ptr's deleter to exist!
	  m_current_image.reset();

	  m_topic = topic;
	  m_tf_filter.reset();

	  if (!m_sub)
	  {
		  m_sub.reset(new image_transport::SubscriberFilter());
	  }

	  if (!topic.empty())
	  {
		  m_sub->subscribe(m_it, topic, 1, image_transport::TransportHints(m_transport_type));

	    if (m_frame.empty())
	    {
	      m_sub->registerCallback(boost::bind(&CRosTopicTexture::callback, this, _1));
	    }
	    else
	    {
	      ROS_ASSERT(m_tf_client);
	      m_tf_filter.reset(new tf::MessageFilter<sensor_msgs::Image>(*m_sub, (tf::Transformer&)*m_tf_client, m_frame, 2, m_nh));
	      m_tf_filter->registerCallback(boost::bind(&CRosTopicTexture::callback, this, _1));
	    }
	  }
	  else
	  {
	    m_sub->unsubscribe();
	  }
}

/**
 * Update texture data
 */
bool srs_ui_but::CRosTopicTexture::update()
{
//	std::cerr << "CRosTopicTexture::update" << std::endl;

	sensor_msgs::Image::ConstPtr image;

	bool new_image = false;
	{
		boost::mutex::scoped_lock lock(m_mutex);

		image = m_current_image;
		new_image = m_new_image;
	}

//	std::cerr << "Image: " << image << ", new_image: " << new_image << std::endl;

	if (!image || !new_image)
	{
		return false;
	}

//	std::cerr << "Texture update. Topic: " << m_topic << std::endl;

	m_new_image = false;

	if (image->data.empty())
	{
		return false;
	}

	// Convert input data
	return m_texture_converter.convert( image );
}

/**
 * Clear data
 */
void srs_ui_but::CRosTopicTexture::clear()
{
	boost::mutex::scoped_lock lock(m_mutex);

//	texture_->unload();
//	texture_->loadImage(empty_image_);

	m_new_image = false;
	m_current_image.reset();

	if (m_tf_filter)
	{
		m_tf_filter->clear();
	}
}

/**
 * On new image data callback
 */
void srs_ui_but::CRosTopicTexture::callback(const sensor_msgs::Image::ConstPtr& image)
{
	boost::mutex::scoped_lock lock(m_mutex);

	m_current_image = image;
	m_new_image = true;

//	std::cerr << "CB: Image: " << m_current_image << ", new_image: " << m_new_image << std::endl;

//	std::cerr << "New image for: " << m_name << std::endl;
}






