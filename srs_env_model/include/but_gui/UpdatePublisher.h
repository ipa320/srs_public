/*
 *******************************************************************************
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 21.2.2012
 * Description:
 *******************************************************************************
 */

#ifndef UPDATEPUBLISHER_H_
#define UPDATEPUBLISHER_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "topics_list.h"

#define BUFFER_SIZE 5


namespace but_gui
{

enum UpdateType
{
  UPDATE_POSE, UPDATE_SCALE, MENU_CLICKED, MOVEMENT_CHANGED, TAG_CHANGED
};

/*
 * This class publishes updates of Interactive Marker
 */
class UpdatePublisher
{
public:
  /*
   * Constructor
   * @param name is Interactive Marker's name
   */
  UpdatePublisher(std::string im_name, int im_type);
  UpdatePublisher()
  {
  }
  /*
   * Destructor
   */
  virtual ~UpdatePublisher()
  {
  }
  /*
   * Gets update topic
   * @param update type is update's type
   */
  std::string getUpdateTopic(int update_type);
  /*
   * Publishes scale changed message
   * @param new_scale is new scale value
   * @param scale_change is scale value change
   */
  void publishScaleChanged(geometry_msgs::Vector3 new_scale, geometry_msgs::Vector3 scale_change);
  /*
   * Publishes pose changed message
   * @param new_pose is new pose value
   * @param pose_change is pose value change
   */
  void publishPoseChanged(geometry_msgs::Pose new_pose, geometry_msgs::Pose pose_change);
  /*
   * Publishes menu clicked message
   * @param title is menu entry title
   * @param state is menu entry state
   */
  void publishMenuClicked(std::string title, interactive_markers::MenuHandler::CheckState state);
  /*
   * Publishes movement changed message
   * @param new_direction is new direction value
   * @param direction_change is direction value change
   * @param new_velocity is new velocity value
   * @param velocity_change is velocity value change
   */
  void publishMovementChanged(geometry_msgs::Quaternion new_direction, geometry_msgs::Quaternion direction_change,
                              float new_velocity, float velocity_change);
  /*
   * Publishes movement changed message
   * @param new_tag is new tag value
   */
  void publishTagChanged(std::string new_tag);

private:
  // Interactive Marker name
  std::string im_name_;
  // Interactive Marker type
  int im_type_;
  // Node handler
  ros::NodeHandle nh_;
  // Publishers
  ros::Publisher scaleChangedPublisher_, poseChangedPublisher_, menuClickedPublisher_, movementChangedPublisher_,
                 tagChangedPublisher_;

};

}

#endif /* UPDATEPUBLISHER_H_ */
