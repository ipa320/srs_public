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

#include "ros_rtt_texture.h"
#include "sensor_msgs/image_encodings.h"

#include <tf/tf.h>

#include <OGRE/OgreTextureManager.h>
#include "OGRE/OgreMaterialManager.h"
#include "OGRE/OgreHardwarePixelBuffer.h"

#include "OGRE/OgreRoot.h"
#include "OGRE/OgreRenderWindow.h"

#define OGRE_TEXTURE_FORMAT Ogre::PF_BYTE_BGRA
#define OGRE_DEPTH_TEXTURE_FORMAT Ogre::PF_FLOAT16_R
#define ROS_IMAGE_FORMAT sensor_msgs::image_encodings::BGRA8
#define BPP 4
#define BPP_DEPTH 2

namespace srs_ui_but
{


CRosRttTexture::CRosRttTexture(unsigned width, unsigned height, Ogre::Camera * camera, bool isDepth /*= false*/ )
: m_materialName("MyRttMaterial")
, width_(width)
, height_(height)
, frame_("/map")
, m_bIsDepth( isDepth )
{
  assert( height > 0 && width > 0 );

  {
    // Set encoding
    current_image_.encoding = ROS_IMAGE_FORMAT;

    // Set image size
    current_image_.width = width;
    current_image_.height = height;

    // Set image row length in bytes (row length * 3 bytes for a color)
    current_image_.step = width * BPP;

#if OGRE_ENDIAN == ENDIAN_BIG
    current_image_.is_bigendian = true;
#else
        current_image_.is_bigendian = false;
#endif

    // Resize data
    current_image_.data.resize( width_ * height_ * BPP);

  }

  Ogre::TextureManager & lTextureManager( Ogre::TextureManager::getSingleton() );
  Ogre::String textureName("RVIZ_CamCast_Texture");
  bool lGammaCorrection( false );
  unsigned int lAntiAliasing( 0 );
  unsigned int lNumMipmaps( 0 );

  if( isDepth )
  {
	  texture_ = lTextureManager.createManual(textureName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		  Ogre::TEX_TYPE_2D, width, height, lNumMipmaps,
		  OGRE_DEPTH_TEXTURE_FORMAT, Ogre::TU_RENDERTARGET, 0, lGammaCorrection, lAntiAliasing);
  }
  else
  {
	  texture_ = lTextureManager.createManual(textureName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
	  		  Ogre::TEX_TYPE_2D, width, height, lNumMipmaps,
	  		  OGRE_TEXTURE_FORMAT, Ogre::TU_RENDERTARGET, 0, lGammaCorrection, lAntiAliasing);
  }

  // Create render target
  Ogre::RenderTexture* lRenderTarget = NULL;

  Ogre::HardwarePixelBufferSharedPtr lRttBuffer = texture_->getBuffer();
  lRenderTarget = lRttBuffer->getRenderTarget();
  lRenderTarget->setAutoUpdated(true);

  // Create and attach viewport

  Ogre::Viewport* lRttViewport1 = lRenderTarget->addViewport(camera, 50, 0.00f, 0.00f, 1.0f, 1.0f);
  lRttViewport1->setAutoUpdated(true);
  Ogre::ColourValue lBgColor1(0.0,0.0,0.0,1.0);
  lRttViewport1->setBackgroundColour(lBgColor1);

  // create a material using this texture.

  //Get a reference on the material manager, which is a singleton.
  Ogre::MaterialManager& lMaterialManager = Ogre::MaterialManager::getSingleton();
  Ogre::MaterialPtr lMaterial = lMaterialManager.create(m_materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::Technique * lTechnique = lMaterial->getTechnique(0);
  Ogre::Pass* lPass = lTechnique->getPass(0);

  if( isDepth )
  {
	  lPass->setLightingEnabled(false);
  }

  Ogre::TextureUnitState* lTextureUnit = lPass->createTextureUnitState();
  lTextureUnit->setTextureName(textureName);

  lTextureUnit->setNumMipmaps(0);
  lTextureUnit->setTextureFiltering(Ogre::TFO_BILINEAR);

  update();
}

CRosRttTexture::~CRosRttTexture()
{
}


void CRosRttTexture::setFrame(const std::string& frame)
{
  frame_ = frame;

  // Set frame id to the image
  current_image_.header.frame_id = frame_;

}



const sensor_msgs::Image & CRosRttTexture::getImage()
{
  boost::mutex::scoped_lock lock(mutex_);

  // Copy texture to the msg image
  update();

  return current_image_;
}

void CRosRttTexture::saveImage(const std::string & filename)
{
  // Copy texture data to the image
  Ogre::Image ogre_image;
  texture_->convertToImage( ogre_image );

  ogre_image.save( filename );
}


void CRosRttTexture::update()
{
  // Copy texture data to the image
  Ogre::Image ogre_image;
  texture_->convertToImage( ogre_image );
  sensor_msgs::Image::_data_type::iterator outputPtr(current_image_.data.begin());

  if( m_bIsDepth )
  {
	  long size = width_ * height_;

	  Ogre::Real * dataPtr( reinterpret_cast< Ogre::Real * >( ogre_image.getData() ) );
	  Ogre::Real * beginPtr( dataPtr );
	  Ogre::Real min( 1000000.0f ), max( -1000000.0f );


	  // Compute depth limits
	  for( long i = 0; i < size; ++i )
	  {
		  if( *dataPtr > max )
			  max = *outputPtr;

		  if( *dataPtr < min )
			  min = *outputPtr;

		  ++dataPtr;
	  }

	  float scale( 1.0 );

	  if( max > min )
	  {
		  scale = 1.0f/ ( max - min );
	  }

	  dataPtr = beginPtr;

	  Ogre::uchar a(255), v;

	  // Copy pixels, set alpha to max...
	  for( long i = 0; i < size; ++i )
	  {
		v = Ogre::uchar( (*dataPtr - min) * scale ); ++dataPtr;
		*outputPtr = v; ++outputPtr;
		*outputPtr = v; ++outputPtr;
		*outputPtr = v; ++outputPtr;
		*outputPtr = a; ++outputPtr;
	  }

  }
  else
  {
	  // Image size in bytes
	  long size = width_ * height_ * BPP;

	  Ogre::uchar * dataPtr( ogre_image.getData() );

	  // Copy data (Can be optimized, of course, but I am too lazy :P )
	  for( long i = 0; i < size; ++i )
	  {
	    *outputPtr = *dataPtr;
	    ++outputPtr;
	    ++dataPtr;
	  }
  }

}


} // namespace srs_ui_but
