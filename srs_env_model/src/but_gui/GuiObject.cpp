/*
 *******************************************************************************
 * $Id: GuiObject.cpp 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 24.11.2011
 *******************************************************************************
 */

#include "but_gui/GuiObject.h"

namespace but_gui
{

void GuiObject::insert()
{
  server->insert(object);
}

void GuiObject::erase()
{
  server->erase(name);
}

string GuiObject::getName()
{
  return name;
}

void GuiObject::setColor(ColorRGBA color_)
{
  color = color_;
}

ColorRGBA GuiObject::getColor()
{
  return color;
}

void GuiObject::setPose(Pose pose_)
{
  pose = pose_;
}

Pose GuiObject::getPose()
{
  return pose;
}

void GuiObject::setScale(Scale scale_)
{
  scale = scale_;
}

Scale GuiObject::getScale()
{
  return scale;
}

void GuiObject::setDescription(string description_)
{
  description = description_;
}

string GuiObject::getDescription()
{
  return description;
}

void GuiObject::setFrameID(string frame_id_)
{
  frame_id = frame_id_;
}

string GuiObject::getFrameID()
{
  return frame_id;
}

void GuiObject::addMovementControls(InteractiveMarker &marker)
{
  marker.controls.push_back(cMoveX);
  marker.controls.push_back(cMoveY);
  marker.controls.push_back(cMoveZ);
}

void GuiObject::addRotationControls(InteractiveMarker &marker)
{
  marker.controls.push_back(cRotateX);
  marker.controls.push_back(cRotateY);
  marker.controls.push_back(cRotateZ);
}

void GuiObject::removeMovementControls(InteractiveMarker &marker, bool rotation)
{
  marker.controls.erase(marker.controls.begin() + baseControlCount, marker.controls.end());
  if (rotation)
    addRotationControls(marker);
}

void GuiObject::removeRotationControls(InteractiveMarker &marker, bool movement)
{
  marker.controls.erase(marker.controls.begin() + baseControlCount, marker.controls.end());
  if (movement)
    addMovementControls(marker);
}

void GuiObject::createControls()
{
  cMoveX.name = "move_x";
  cMoveX.orientation.w = 1;
  cMoveX.orientation.x = 1;
  cMoveX.orientation.y = 0;
  cMoveX.orientation.z = 0;
  cMoveX.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;

  cMoveY.name = "move_y";
  cMoveY.orientation.w = 1;
  cMoveY.orientation.x = 0;
  cMoveY.orientation.y = 0;
  cMoveY.orientation.z = 1;
  cMoveY.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;

  cMoveZ.name = "move_z";
  cMoveZ.orientation.w = 1;
  cMoveZ.orientation.x = 0;
  cMoveZ.orientation.y = 1;
  cMoveZ.orientation.z = 0;
  cMoveZ.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;

  cRotateX.name = "rotate_x";
  cRotateX.orientation.w = 1;
  cRotateX.orientation.x = 1;
  cRotateX.orientation.y = 0;
  cRotateX.orientation.z = 0;
  cRotateX.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;

  cRotateY.name = "rotate_y";
  cRotateY.orientation.w = 1;
  cRotateY.orientation.x = 0;
  cRotateY.orientation.y = 0;
  cRotateY.orientation.z = 1;
  cRotateY.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;

  cRotateZ.name = "rotate_z";
  cRotateZ.orientation.w = 1;
  cRotateZ.orientation.x = 0;
  cRotateZ.orientation.y = 1;
  cRotateZ.orientation.z = 0;
  cRotateZ.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
}

void GuiObject::createMeasureControl()
{
  Marker measureMarker;
  measureMarker.type = Marker::LINE_LIST;
  measureMarker.color.r = color.g;
  measureMarker.color.g = color.b;
  measureMarker.color.b = color.r;
  measureMarker.color.a = 0.0;
  measureMarker.scale.x = 0.01;
  Point p;
  // Z
  double sx = scale.x / 2;
  double sy = scale.y / 2;
  double sz = scale.z / 2;
  p.x = -sx;
  p.y = -sy;
  p.z = -sz;
  measureMarker.points.push_back(p);
  p.z = sz;
  measureMarker.points.push_back(p);
  // Y
  p.x = -sx;
  p.y = sy;
  p.z = -sz;
  measureMarker.points.push_back(p);
  p.y = -sy;
  measureMarker.points.push_back(p);
  // X
  p.x = sx;
  p.y = -sy;
  p.z = -sz;
  measureMarker.points.push_back(p);
  p.x = -sx;
  measureMarker.points.push_back(p);

  Marker measureText;
  ostringstream text_x, text_y, text_z;
  text_x << scale.x << "m";
  text_y << scale.y << "m";
  text_z << scale.z << "m";
  measureText.type = Marker::TEXT_VIEW_FACING;
  measureText.color = measureMarker.color;
  measureText.scale.z = MEASURE_TEXT_SIZE;
  // Z
  measureText.pose.position.x = -sx - 0.2;
  measureText.pose.position.y = -sy - 0.2;
  measureText.pose.position.z = 0;
  measureText.text = text_z.str();
  measureControl.markers.push_back(measureText);
  // Y
  measureText.pose.position.x = -sx - 0.2;
  measureText.pose.position.y = 0;
  measureText.pose.position.z = -sz - 0.2;
  measureText.text = text_y.str();
  measureControl.markers.push_back(measureText);
  // X
  measureText.pose.position.x = 0;
  measureText.pose.position.y = -sy - 0.2;
  measureText.pose.position.z = -sz - 0.2;
  measureText.text = text_x.str();
  measureControl.markers.push_back(measureText);

  measureControl.name = "measure_control";
  measureControl.markers.push_back(measureMarker);
  measureControl.interaction_mode = InteractiveMarkerControl::NONE;
  measureControl.always_visible = true;
}

void GuiObject::createDescriptionControl()
{

  Marker descriptionMarker;
  descriptionMarker.type = Marker::TEXT_VIEW_FACING;
  descriptionMarker.text = description;
  descriptionMarker.scale.z = 0.2;
  descriptionMarker.color.r = color.b;
  descriptionMarker.color.g = color.r;
  descriptionMarker.color.b = color.g;
  descriptionMarker.color.a = 0.0;
  descriptionMarker.pose.position.z = object.scale / 2 + 0.2;

  descriptionControl.name = "description_control";
  descriptionControl.markers.push_back(descriptionMarker);
  descriptionControl.interaction_mode = InteractiveMarkerControl::NONE;
  descriptionControl.orientation_mode = InteractiveMarkerControl::FIXED;
  descriptionControl.always_visible = true;
}

void GuiObject::changeColor(ColorRGBA color_)
{
  color = color_;
  control.markers.clear();
  object.controls.clear();
  erase();
  create();
  insert();
}

float maxScale(Scale scale)
{
  if (scale.x > scale.y)
  {
    if (scale.x > scale.z)
    {
      return scale.x;
    }
    else
    {
      return scale.z;
    }
  }
  else
  {
    if (scale.y > scale.z)
    {
      return scale.y;
    }
    else
    {
      return scale.z;
    }
  }
}

}
