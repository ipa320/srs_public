/*
 *******************************************************************************
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 21.2.2012
 * Description:
 *******************************************************************************
 */

#include "but_gui/UpdatePublisher.h"

namespace but_gui
{

UpdatePublisher::UpdatePublisher(std::string name, int type)
{
  im_name_ = name;
  im_type_ = type;

  poseChangedPublisher_ = nh_.advertise<srs_env_model::PoseChanged> (BUT_PoseChanged_TOPIC(im_name_), BUFFER_SIZE);
  scaleChangedPublisher_ = nh_.advertise<srs_env_model::ScaleChanged> (BUT_ScaleChanged_TOPIC(im_name_), BUFFER_SIZE);
  menuClickedPublisher_ = nh_.advertise<srs_env_model::MenuClicked> (BUT_MenuClicked_TOPIC(im_name_), BUFFER_SIZE);

  switch (im_type_)
  {
    case srs_env_model::PrimitiveType::BILLBOARD:
      movementChangedPublisher_ = nh_.advertise<srs_env_model::MovementChanged> (BUT_MovementChanged_TOPIC(im_name_),
                                                                                 BUFFER_SIZE);
      break;
    case srs_env_model::PrimitiveType::PLANE:
      tagChangedPublisher_ = nh_.advertise<srs_env_model::TagChanged> (BUT_TagChanged_TOPIC(im_name_), BUFFER_SIZE);
      break;
  }
}

std::string UpdatePublisher::getUpdateTopic(int update_type)
{
  std::string topic;
  switch (update_type)
  {
    case UPDATE_SCALE:
      topic = scaleChangedPublisher_.getTopic();
      break;
    case UPDATE_POSE:
      topic = poseChangedPublisher_.getTopic();
      break;
    case MENU_CLICKED:
      topic = menuClickedPublisher_.getTopic();
      break;
    case MOVEMENT_CHANGED:
      topic = movementChangedPublisher_.getTopic();
      break;
    case TAG_CHANGED:
      topic = tagChangedPublisher_.getTopic();
      break;
    default:
      topic = "";
      break;
  }

  if (topic == "")
    ROS_WARN("No update topic of type %d for marker of type %d!", update_type, im_type_);
  return topic;
}

void UpdatePublisher::publishScaleChanged(geometry_msgs::Vector3 new_scale, geometry_msgs::Vector3 scale_change)
{
  srs_env_model::ScaleChanged scaleChangedMsg;
  scaleChangedMsg.marker_name = im_name_;
  scaleChangedMsg.new_scale = new_scale;
  scaleChangedMsg.scale_change = scale_change;
  scaleChangedPublisher_.publish(scaleChangedMsg);
}

void UpdatePublisher::publishPoseChanged(geometry_msgs::Pose new_pose, geometry_msgs::Pose pose_change)
{
  srs_env_model::PoseChanged poseChangedMsg;
  poseChangedMsg.marker_name = im_name_;
  poseChangedMsg.new_pose = new_pose;
  poseChangedMsg.pose_change = pose_change;
  poseChangedPublisher_.publish(poseChangedMsg);
}

void UpdatePublisher::publishMenuClicked(std::string title, interactive_markers::MenuHandler::CheckState state)
{
  srs_env_model::MenuClicked menuClickedMsg;
  menuClickedMsg.marker_name = im_name_;
  menuClickedMsg.menu_title = title;
  menuClickedMsg.state = state;
  menuClickedPublisher_.publish(menuClickedMsg);
}

void UpdatePublisher::publishMovementChanged(geometry_msgs::Quaternion new_direction,
                                             geometry_msgs::Quaternion direction_change, float new_velocity,
                                             float velocity_change)
{
  srs_env_model::MovementChanged movementChangedMsg;
  movementChangedMsg.marker_name = im_name_;
  movementChangedMsg.new_direction = new_direction;
  movementChangedMsg.direction_change = direction_change;
  movementChangedMsg.new_velocity = new_velocity;
  movementChangedMsg.velocity_change = velocity_change;
  movementChangedPublisher_.publish(movementChangedMsg);
}

void UpdatePublisher::publishTagChanged(std::string new_tag)
{
  srs_env_model::TagChanged tagChangedMsg;
  tagChangedMsg.marker_name = im_name_;
  tagChangedMsg.new_tag = new_tag;
  tagChangedPublisher_.publish(tagChangedMsg);
}

}
