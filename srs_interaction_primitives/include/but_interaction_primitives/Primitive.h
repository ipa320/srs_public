/******************************************************************************
 * \file
 *
 * $Id: Primitive.h 676 2012-04-19 18:32:07Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 24/11/2011
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

#ifndef PRIMITIVE_H_
#define PRIMITIVE_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/tools.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/ColorRGBA.h>
#include <OGRE/OgreVector3.h>
#include <arm_navigation_msgs/Shape.h>

#include <srs_interaction_primitives/PrimitiveType.h>
#include "UpdatePublisher.h"

#define MEASURE_TEXT_SCALE 0.2
#define MEASURE_TEXT_MAX_SIZE 0.3
#define MEASURE_TEXT_MIN_SIZE 0.1

#define GRASP_ARROW_LENGTH 0.25
#define GRASP_ARROW_WIDTH 0.2
#define GRASP_TEXT_OFFSET 0.1
#define GRASP_TEXT_SIZE 0.1
#define GRASP_POINT_SCALE 0.05
#define GRASP_TRANSPARENCY 0.3

using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std_msgs;
using namespace interactive_markers;
using namespace boost;
using namespace std;

namespace but_interaction_primitives
{

/**
 * @brief Pre-grasp positions ids.
 */
enum
{
  _, GRASP_1, GRASP_2, GRASP_3, GRASP_4, GRASP_5, GRASP_6
};

/**
 * @brief Object resource types.
 */
enum ObjectResource
{
  SHAPE, RESOURCE_FILE
};

/**
 * @brief Pre-grasp position.
 */
struct PreGraspPosition
{
  string name;
  bool enabled;
  Pose pose;

  PreGraspPosition()
  {
    enabled = false;
  }
};

typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;
typedef geometry_msgs::Vector3 Scale;

/**
 * @brief Returns maximal scale value
 * @param scale is vector of scale values
 */
float maxScale(Vector3 scale);

/**
 * @brief Returns minimal scale value
 * @param scale is vector of scale values
 */
float minScale(Vector3 scale);

/**
 * This class is parent class for BUT GUI primitives
 *
 * @author Tomas Lokaj
 */
class Primitive
{
public:
  /**
   * @brief Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this primitive
   */
  Primitive(InteractiveMarkerServerPtr server, string frame_id, string name, int type);

  /**
   * @brief Constructor
   */
  Primitive()
  {
  }

  /**
   * @brief Destructor
   */
  virtual ~Primitive()
  {
    server_->erase(name_);
    server_->applyChanges();
  }

  /**
   * @brief Insert primitive into Interactive Marker Server
   */
  virtual void insert();

  /**
   * @brief Erase primitive from Interactive Marker Server
   */
  void erase();

  /**
   * @brief Change primitive's color
   * @param color is primitives's color
   */
  void changeColor(ColorRGBA color);

  /**
   * @brief Sets type of this primitive.
   * @param type is primitive's type
   */
  void setPrimitiveType(int type)
  {
    primitive_type_ = type;
    delete updatePublisher_;
    updatePublisher_ = new UpdatePublisher(name_, primitive_type_);
  }

  /**
   * @brief Gets type of this primitive.
   * @return primitives type
   */
  int getPrimitiveType()
  {
    return primitive_type_;
  }

  /**
   * @brief Gets update topic
   * @param update_type is update's type
   */
  string getUpdateTopic(int update_type)
  {
    return updatePublisher_->getUpdateTopic(update_type);
  }

  /**
   * @brief Gets primitive's name
   * @return primitive's name
   */
  string getName()
  {
    return name_;
  }

  /**
   * @brief Sets color of the primitive
   * @param color is primitive's color
   */
  void setColor(ColorRGBA color)
  {
    color_ = color;
  }

  /**
   * @brief Gets primitive's color
   * @return primitive's color
   */
  ColorRGBA getColor()
  {
    return color_;
  }

  /**
   * @brief Sets position and orientation to object
   * @param pose is primitive's position and orientation
   */
  void setPose(Pose pose)
  {
    Pose pose_change;
    pose_change.position.x = pose.position.x - pose_.position.x;
    pose_change.position.y = pose.position.y - pose_.position.y;
    pose_change.position.z = pose.position.z - pose_.position.z;
    pose_change.orientation.x = pose.orientation.x - pose_.orientation.x;
    pose_change.orientation.y = pose.orientation.y - pose_.orientation.y;
    pose_change.orientation.z = pose.orientation.z - pose_.orientation.z;
    pose_change.orientation.w = pose.orientation.w - pose_.orientation.w;
    pose_ = pose;
    object_.pose = pose_;
    updatePublisher_->publishPoseChanged(pose_, pose_change);
  }

  /**
   * @brief Gets primitive's position and orientation
   * @return primitive's position and orientation
   */
  Pose getPose()
  {
    return pose_;
  }

  /**
   * @brief Sets scale to object
   * @param scale is primitive's scale
   */
  void setScale(Vector3 scale)
  {
    Vector3 scale_change;
    scale_change.x = scale.x - scale_.x;
    scale_change.y = scale.y - scale_.y;
    scale_change.z = scale.z - scale_.z;
    scale_ = scale;
    object_.scale = but_interaction_primitives::maxScale(scale_);
    updatePublisher_->publishScaleChanged(scale_, scale_change);
  }

  /**
   * @brief Gets primitive's scale
   * @return primitive's scale
   */
  Vector3 getScale()
  {
    return scale_;
  }

  /**
   * @brief Sets description to object
   * @param description is primitive's description
   */
  void setDescription(string description)
  {
    description_ = description;
  }

  /**
   * Gets primitive's description
   * @return primitive's description
   */
  string getDescription()
  {
    return description_;
  }

  /**
   * @brief Sets fixed frame to object
   * @param frame_id is primitive's fixed frame
   */
  void setFrameID(string frame_id)
  {
    frame_id_ = frame_id;
  }

  /**
   * @brief Gets primitive's fixed frame
   * @return primitive's fixed frame
   */
  string getFrameID()
  {
    return frame_id_;
  }

  /**
   * @brief Gets plane's normal.
   * @return plane's normal
   */
  Ogre::Vector3 getNormal()
  {
    return normal_;
  }

  /**
   * @brief Sets plane's normal.
   * @param normal is plane's normal
   */
  void setNormal(Ogre::Vector3 normal)
  {
    normal_ = normal;
  }

  /**
   * @brief Sets plane's normal.
   * @param normal is plane's normal
   */
  void setNormal(geometry_msgs::Vector3 normal)
  {
    normal_ = Ogre::Vector3(normal.x, normal.y, normal.z);
  }

  /**
   * @brief Sets direction to the billboard
   * @param direction is billboard's direction
   */
  void setDirection(Quaternion direction)
  {
    if (primitive_type_ != srs_interaction_primitives::PrimitiveType::BILLBOARD)
    {
      ROS_WARN("This is object is not a billboard, you cannot set direction!");
      return;
    }

    geometry_msgs::Quaternion direction_change;
    direction_change.x = direction.w - direction_.x;
    direction_change.y = direction.y - direction_.y;
    direction_change.z = direction.z - direction_.z;
    direction_change.w = direction.w - direction_.w;

    updatePublisher_->publishMovementChanged(direction, direction_change, velocity_, 0.0f);

    direction_ = direction;
  }

  /**
   * @brief Gets billboard's movement direction
   * @return billboard's movement direction
   */
  Quaternion getDirection()
  {
    if (primitive_type_ != srs_interaction_primitives::PrimitiveType::BILLBOARD)
      ROS_WARN("This is object is not a billboard, you cannot get direction!");
    return direction_;
  }

  /**
   * @brief Sets velocity to the billboard
   * @param velocity is billboard's velocity
   */
  void setVelocity(double velocity)
  {
    if (primitive_type_ != srs_interaction_primitives::PrimitiveType::BILLBOARD)
    {
      ROS_WARN("This is object is not a billboard, you cannot set velocity!");
      return;
    }
    updatePublisher_->publishMovementChanged(direction_, geometry_msgs::Quaternion(), velocity, velocity - velocity_);
    velocity_ = velocity;
  }

  /**
   * @brief Sets PlanePolygon's polygon
   * @param polygon is PlanePolygon's polygon
   */
  void setPolygon(Polygon polygon)
  {
    polygon_ = polygon;
  }

  /**
   * @brief Gets billboard's movement velocity
   * @return billboard's movement velocity
   */
  double getVelocity()
  {
    if (primitive_type_ != srs_interaction_primitives::PrimitiveType::BILLBOARD)
      ROS_WARN("This is object is not a billboard, you cannot get velocity!");
    return velocity_;
  }

  /**
   * @brief Sets resource file with primitive's model (and material).
   * @param resource path to the resource file
   */
  void setResource(string resource)
  {
    object_resource_ = RESOURCE_FILE;
    resource_ = resource;
  }

  /**
   * @brief Sets usage of defined material or color
   * @param use_material material (true), color (false)
   */
  void setUseMaterial(bool use_material)
  {
    use_material_ = use_material;
  }

  /**
   * @brief Sets shape of the mesh
   * @param shape is shape
   */
  void setShape(arm_navigation_msgs::Shape shape)
  {
    object_resource_ = SHAPE;
    shape_ = shape;
  }

  /**
   * @brief Sets specified pre-grasp position
   * @param pos_num is specified position number (1-6)
   * @param pose is pre-grasp position and orientation
   */
  void addPreGraspPosition(int pos_id, Pose pose);

  /**
   * @brief Removes specified pre-grasp position
   * @param pos_num is specified position number (1-6)
   */
  void removePreGraspPosition(int pos_id);

  /**
   * @brief Updates visible controls
   */
  void updateControls();

  /**
   * @brief Returns control
   * @param name is controls name
   */
  InteractiveMarkerControl * getControl(string name);

  /**
   * @brief Default callback for objects
   */
  virtual void defaultCallback(const InteractiveMarkerFeedbackConstPtr &feedback);

protected:
  /**
   * @brief Create Object
   */
  virtual void create()
  {
  }

  /**
   * @brief Create menu
   */
  virtual void createMenu()
  {
  }

  /**
   * @brief Adds description controls
   */
  void addDescriptionControl();

  /**
   * @brief Adds measure controls
   */
  void addMeasureControl();

  /**
   * @brief Adds movement controls
   * @param marker is Interactive marker
   */
  void addMovementControls();

  /**
   * @brief Adds rotation controls
   */
  void addRotationControls();

  /**
   * @brief Removes movement controls
   */
  void removeMovementControls();

  /**
   * @brief Removes rotation controls
   */
  void removeRotationControls();

  /**
   * @brief Removes measure control
   */
  void removeMeasureControl();

  /**
   * Removes description control
   */
  void removeDescriptionControl();

  /**
   * Removes control with specified name
   * @param name is control's name
   */
  void removeControl(string name);

  /**
   * @brief Resets object
   */
  void clearObject();

  /**
   * @brief Adds scale controls
   */
  void addScaleControls();

  /**
   * @brief Updates scale controls
   */
  virtual void updateScaleControls();

  /**
   * @brief Removes scale controls
   */
  void removeScaleControls();

  /**
   * @brief Adds grasping positions
   */
  void addPregraspPositions();

  /**
   * @brief Removes grasping positions
   */
  void removePregraspPositions();

  /**
   * @brief Adds trajectory controls
   */
  void addTrajectoryControls();

  /**
   * @brief Removes trajectory control
   */
  void removeTrajectoryControls();

  /**
   * @brief Scale controls feedback
   */
  void scaleFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  Vector3 min_size_, max_size_;
  int primitive_type_;
  InteractiveMarker object_;
  InteractiveMarkerControl control_, descriptionControl_, measureControl_, scaleControl_, trajectoryControl_;
  InteractiveMarkerControl moveXControl_, moveYControl_, moveZControl_, rotateXControl_, rotateYControl,
                           rotateZControl_;
  InteractiveMarkerControl pregrasp1Control_, pregrasp2Control_, pregrasp3Control_, pregrasp4Control_,
                           pregrasp5Control_, pregrasp6Control_;
  MenuHandler menu_handler_;

  // Common attributes
  string name_, description_, frame_id_;
  InteractiveMarkerServerPtr server_;
  Pose pose_;
  Vector3 scale_;
  ColorRGBA color_, color_green_a01_;

  // Billboard's attributes
  double velocity_;
  Quaternion direction_;

  // Object's attributes
  ObjectResource object_resource_;
  Marker mesh_;
  arm_navigation_msgs::Shape shape_;
  string resource_;
  bool use_material_;
  bool move_arm_to_pregrasp_onclick_;

  // PlanePolygon's attributes
  Ogre::Vector3 normal_;
  Polygon polygon_;

  UpdatePublisher *updatePublisher_;

  bool show_movement_control_, show_scale_control_, show_rotation_control_, show_measure_control_,
       show_description_control_, show_pregrasp_control_, show_trajectory_control_;
  bool menu_created_;
  PreGraspPosition pregrasp1_, pregrasp2_, pregrasp3_, pregrasp4_, pregrasp5_, pregrasp6_;

};
}
#endif /* PRIMITIVE_H_ */
