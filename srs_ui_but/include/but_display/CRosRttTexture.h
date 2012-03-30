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

#ifndef RVIZ_ROS_RTT_TEXTURE_H
#define RVIZ_ROS_RTT_TEXTURE_H

#include <sensor_msgs/Image.h>

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreImage.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <tf/message_filter.h>

#include <stdexcept>

namespace tf
{
class TransformListener;
}

namespace rviz
{

typedef std::vector<std::string> V_string;

class UnsupportedImageEncoding : public std::runtime_error
{
public:
  UnsupportedImageEncoding(const std::string& encoding)
  : std::runtime_error("Unsupported image encoding [" + encoding + "]")
  {}
};

class CRosRttTexture
{
public:
  CRosRttTexture(unsigned width, unsigned height, Ogre::Camera * camera);
  ~CRosRttTexture();

  void setFrame(const std::string& frame);

  const Ogre::TexturePtr& getTexture() const { return texture_; }
  const sensor_msgs::Image & getImage();

  uint32_t getWidth() const { return width_; }
  uint32_t getHeight() const { return height_; }

  const std::string& getMaterialName() const { return m_materialName; }

  bool hasData() const { return current_image_.width > 0 && current_image_.height > 0; }

  void saveImage(const std::string & filename);

protected:
  void update();

  sensor_msgs::Image current_image_;
  Ogre::TexturePtr texture_;

  Ogre::String m_materialName;

  uint32_t width_;
  uint32_t height_;

  std::string frame_;

  boost::mutex mutex_;

 };

} // namespace rviz

// RVIZ_ROS_RTT_TEXTURE_H
#endif

