/*
 *  Based on visualization/rviz/src/rviz/image/ros_image_texture.h
 *
 *  Added rtt image initialization
 *
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

