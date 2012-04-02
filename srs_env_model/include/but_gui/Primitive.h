/******************************************************************************
 * \file
 *
 * $Id: Primitive.h 397 2012-03-29 12:50:30Z spanel $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd.mm.2012
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

#ifndef CPRIMITIVE_H_
#define CPRIMITIVE_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/tools.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <but_gui/UpdatePublisher.h>
#include <srs_env_model/PrimitiveType.h>
#include <arm_navigation_msgs/Shape.h>

#define MEASURE_TEXT_SCALE 0.2
#define MEASURE_TEXT_MAX_SIZE 0.3
#define MEASURE_TEXT_MIN_SIZE 0.1
#define GRASP_ARROW_SCALE 1.2
#define GRASP_ARROW_LENGTH 0.2

using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std_msgs;
using namespace interactive_markers;
using namespace boost;
using namespace std;

namespace but_gui
{

enum
{
  GRASP_1, GRASP_2, GRASP_3, GRASP_4, GRASP_5, GRASP_6
};

enum ObjectResource
{
  SHAPE, RESOURCE_FILE
};

struct GraspPosition
{
  string name;
  bool enabled;
  Vector3 position;

  GraspPosition()
  {
    enabled = false;
  }
};

typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;
typedef Vector3 Scale;
/*
 * Returns maximal scale value
 * @param scale is vector of scale values
 */
float maxScale(Vector3);
/*
 * Returns minimal scale value
 * @param scale is vector of scale values
 */
float minScale(Vector3);

class Primitive
{
public:
  /**
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this primitive
   */
  Primitive(InteractiveMarkerServerPtr server, string frame_id, string name, int type);
  /**
   * Constructor
   */
  Primitive()
  {
  }
  /*
   * Destructor
   */
  virtual ~Primitive()
  {
    server_->erase(name_);
    server_->applyChanges();
  }

  /*
   * Insert object into Interactive Marker Server
   */
  virtual void insert();
  /*
   * Erase object from Interactive Marker Server
   */
  void erase();
  /*
   * Change object's color
   */
  void changeColor(ColorRGBA color);

  //Getters and setters

  /*
   * sets type of this primitive.
   */
  void setPrimitiveType(int type)
  {
    primitive_type_ = type;
  }
  /*
   * Gets type of this primitive.
   */
  int getPrimitiveType()
  {
    return primitive_type_;
  }
  /*
   * Gets update topic
   * @param update_type is update's type
   */
  string getUpdateTopic(int update_type);
  /*
   * Gets object's name
   */
  string getName();
  /*
   * Sets color to object
   * @param color is object's color
   */
  void setColor(ColorRGBA color);
  /*
   * Gets object's color
   */
  ColorRGBA getColor();
  /*
   * Sets position and orientation to object
   * @param pose is object's position and orientation
   */
  void setPose(Pose pose);
  /*
   * Gets object's position and orientation
   */
  Pose getPose();
  /*
   * Sets scale to object
   * @param scale is object's scale
   */
  void setScale(Vector3 scale);
  /*
   * Gets object's scale
   */
  Vector3 getScale();
  /*
   * Sets description to object
   * @param description is object's description
   */
  void setDescription(string description);
  /*
   * Gets object's description
   */
  string getDescription();
  /*
   * Sets fixed frame to object
   * @param frame_id is object's fixed frame
   */
  void setFrameID(string frame_id);
  /*
   * Gets object's fixed frame
   */
  string getFrameID();

  /*
   * Sets direction to the billboard
   * @param direction is billboard's direction
   */
  void setDirection(Quaternion direction);
  /*
   * Gets billboard's direction
   */
  Quaternion getDirection();
  /*
   * Sets velocity to the billboard
   * @param velocity is billboard's velocity
   */
  void setVelocity(double velocity);
  /*
   * Gets billboard's velocity
   */
  double getVelocity();
  /*
   * Sets resource file with object's model (and material).
   * @param resource path to the resource file
   */
  void setResource(string resource);
  /*
   * Sets usage of defined material or color
   * @param use_material material (true), color (false)
   */
  void setUseMaterial(bool use_material);
  /*
   * Sets shape of the mesh
   * @param shape is shape
   */
  void setShape(arm_navigation_msgs::Shape shape);
  /*
   * Sets specified grasping position
   * @param pos_num is specified position number (1-6)
   * @position is grasping position
   */
  void setGraspingPosition(int pos_id, Vector3 position);
  /*
   * Removes specified grasping position
   * @param pos_num is specified position number (1-6)
   */
  void removeGraspingPosition(int pos_id);
  /*
   * Updates visible controls
   */
  void updateControls();

  /*
   * Returns control
   * @param name is controls name
   */
  InteractiveMarkerControl * getControl(string name);
  /*
   * Default callback for objects
   */
  virtual void defaultCallback(const InteractiveMarkerFeedbackConstPtr &feedback);

protected:
  /*
   * Create Object
   */
  virtual void create()
  {
  }
  /*
   * Create menu
   */
  virtual void createMenu()
  {
  }
  /*
   * Adds measure controls
   */
  void addDescriptionControl();
  /*
   * Adds measure controls
   */
  void addMeasureControl();
  /*
   * Adds movement controls
   * @param marker is Interactive marker
   */
  void addMovementControls();
  /*
   * Adds rotation controls
   */
  void addRotationControls();
  /*
   * Removes movement controls
   */
  void removeMovementControls();
  /*
   * Removes rotation controls
   */
  void removeRotationControls();
  /*
   * Removes measure control
   */
  void removeMeasureControl();
  /*
   * Removes description control
   */
  void removeDescriptionControl();
  /*
   * Removes control with specified name
   * @param name is control's name
   */
  void removeControl(string name);
  /*
   * Resets object
   */
  void clearObject();
  /*
   * Adds scale controls
   */
  void addScaleControls();
  /*
   * Updates scale controls
   */
  virtual void updateScaleControls();
  /*
   * Removes scale controls
   */
  void removeScaleControls();
  /*
   * Adds grasping positions
   */
  void addPregraspPositions();
  /*
   * Removes grasping positions
   */
  void removePregraspPositions();
  /*
   * Adds trajectory controls
   */
  void addTrajectoryControls();
  /*
   * Removes trajectory control
   */
  void removeTrajectoryControls();
  /*
   * Scale controls feedback
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
  ColorRGBA color_, color_green_a02_;

  // Billboard's attributes
  double velocity_;
  Quaternion direction_;

  // Object's attributes
  ObjectResource object_resource_;
  Marker mesh_;
  arm_navigation_msgs::Shape shape_;
  string resource_;
  bool use_material_;

  UpdatePublisher *updatePublisher_;

  bool show_movement_control_, show_scale_control_, show_rotation_control_, show_measure_control_,
       show_description_control_, show_pregrasp_control_, show_trajectory_control_;
  bool menu_created_;
  GraspPosition pregrasp1_, pregrasp2_, pregrasp3_, pregrasp4_, pregrasp5_, pregrasp6_;

};
}
#endif /* CPRIMITIVE_H_ */
