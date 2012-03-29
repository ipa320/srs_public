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

#ifndef BUT_DISTANCE_INDICATOR_H_
#define BUT_DISTANCE_INDICATOR_H_

#include <rviz/display.h>
#include <rviz/view_controller.h>
#include "rviz/properties/forwards.h"
#include "rviz/properties/property.h"
#include "rviz/properties/edit_enum_property.h"
#include "rviz/properties/property_manager.h"
#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include "rviz/helpers/color.h"
#include <rviz/render_panel.h>
#include <rviz/window_manager_interface.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <ogre_tools/movable_text.h>
#include <math.h>
#include <but_ogre_tools/static_text.h>

#define DEFAULT_GRIPPER_LINK "/sdh_palm_link"

using namespace std;

namespace rviz
{

/*
 * This tool displays distances around specified robot's link.
 */
class CButDistanceIndicator : public rviz::Display

{
public:
  /*
   * Constructor
   */
  CButDistanceIndicator(const string & name, rviz::VisualizationManager * manager);

  /*
   * Destructor
   */
  virtual ~CButDistanceIndicator();

  // Overrides from Display
  virtual void targetFrameChanged()
  {
  }
  virtual void fixedFrameChanged()
  {
  }

  /*
   * Create display properties
   */
  virtual void createProperties();

  /*
   * Update display
   */
  virtual void update(float wall_dt, float ros_dt);

protected:
  /*
   * Create geometry (includes scene node initialization)
   */
  bool createGeometry();

  /*
   * Destroy geometry
   */
  void destroyGeometry();

  /*
   * Display enabled
   */
  virtual void onEnable();

  /*
   * Display disabled
   */
  virtual void onDisable();

  /*
   * Get link property
   */
  const string getLinkString()
  {
    return robot_link_;
  }

  /*
   * Get color property
   */
  Color getColor()
  {
    return color_;
  }

  /*
   * Get aplha property
   */
  float getAlpha()
  {
    return alpha_;
  }

  /*
   * Get thickness property
   */
  float getThickness()
  {
    return thickness_;
  }

  /*
   * Get levels property
   */
  int getLevels()
  {
    return levels_;
  }

  /*
   * Get radius property
   */
  float getRadius()
  {
    return radius_;
  }

  /*
   * Get show distance property
   */
  bool getShowDistance()
  {
    return show_distance_;
  }

  /*
   * Set link property
   */
  void setLinkString(string link);

  /*
   * Set color property
   */
  void setColor(Color color);

  /*
   * Set alpha property
   */
  void setAlpha(float alpha);

  /*
   * Set thickness property
   */
  void setThickness(float thickness)
  {
    thickness_ = thickness;
    propertyChanged(m_property_thickness_);
  }

  /*
   * Set show distance property
   */
  void setShowDistance(bool show_distance)
  {
    show_distance_ = show_distance;
  }

  /*
   * Set levels property
   */
  void setLevels(int levels)
  {
    if (levels < 1)
      levels_ = 1;
    else
      levels_ = levels;
    propertyChanged(m_property_levels_);
  }

  /*
   * Set radius property
   */
  void setRadius(float radius)
  {
    radius_ = radius;
    propertyChanged(m_property_radius_);
  }

  // Link
  string robot_link_;

  // Scene node
  Ogre::SceneNode * m_sceneNode_, *m_sceneNode_flipped_;

  // Geometry manual object
  Ogre::ManualObject * circle_manual_object_;

  // Material
  Ogre::MaterialPtr material_;

  // Display properties
  ColorPropertyWPtr m_property_color_;
  TFFramePropertyWPtr m_property_link_;
  FloatPropertyWPtr m_property_alpha_, m_property_thickness_, m_property_radius_;
  IntPropertyWPtr m_property_levels_;
  BoolPropertyWPtr m_show_distance_property_;

  // Circle properties
  float radius_, thickness_, accuracy_;
  unsigned int levels_;

  // Line color
  Color color_;

  // Line aplha
  float alpha_;

  bool show_distance_;

};
}
#endif /* BUT_DISTANCE_INDICATOR_H_ */
