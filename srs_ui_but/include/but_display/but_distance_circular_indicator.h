/******************************************************************************
 * \file
 *
 * $Id: but_distance_circular_indicator.h 556 2012-04-11 16:10:40Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 11/04/2012
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

#ifndef BUT_DISTANCE_CIRCULAR_INDICATOR_H_
#define BUT_DISTANCE_CIRCULAR_INDICATOR_H_

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

/**
 * @brief This tool displays distances around specified robot's link.
 *
 * @author Tomas Lokaj
 */
class CButDistanceCircularIndicator : public rviz::Display

{
public:
  /**
   * @brief Constructor
   */
  CButDistanceCircularIndicator(const string & name, rviz::VisualizationManager * manager);

  /**
   * @brief Destructor
   */
  virtual ~CButDistanceCircularIndicator();

  /**
   * @brief Overriden method from Display
   */
  virtual void targetFrameChanged()
  {
  }
  /**
   * @brief Overriden method from Display
   */
  virtual void fixedFrameChanged()
  {
  }

  /**
   * @brief Creates display properties
   */
  virtual void createProperties();

  /**
   * @brief Updates display
   */
  virtual void update(float wall_dt, float ros_dt);

protected:
  /**
   * @brief Creates geometry (includes scene node initialization)
   */
  bool createGeometry();

  /**
   * @brief Destroys geometry
   */
  void destroyGeometry();

  /**
   * @brief Display is enabled
   */
  virtual void onEnable();

  /**
   * @brief is Display disabled
   */
  virtual void onDisable();

  /**
   * @brief Gets link property
   * @return robot's link
   */
  const string getLinkString()
  {
    return robot_link_;
  }

  /**
   * @brief Sets orientation property
   * @return orientation
   */
  Ogre::Vector3 getOrientation()
  {
    return orientation_;
  }

  /**
   * @brief Gets color property
   * @return color
   */
  Color getColor()
  {
    return color_;
  }

  /**
   * Gets aplha property
   * @return alpha
   */
  float getAlpha()
  {
    return alpha_;
  }

  /**
   * @brief Gets thickness property
   * @return thickness
   */
  float getThickness()
  {
    return thickness_;
  }

  /**
   * @brief Gets levels property
   * @return levels
   */
  int getLevels()
  {
    return levels_;
  }

  /**
   * @brief Gets radius property
   * @return radius
   */
  float getRadius()
  {
    return radius_;
  }

  /**
   * @brief Gets show distance property
   * @return show_distance
   */
  bool getShowDistance()
  {
    return show_distance_;
  }

  /**
   * @brief Sets link property
   * @param link is new link
   */
  void setLinkString(string link)
  {
    if (isEnabled())
      m_sceneNode_->setVisible(true);
    robot_link_ = link;
    propertyChanged(m_property_link_);
  }

  /**
   * @brief Sets orientation property
   * @param orientation is new orientation
   */
  void setOrientation(Ogre::Vector3 orientation)
  {
    orientation_ = orientation;
    propertyChanged(m_property_orientation_);
  }

  /**
   * @brief Sets color property
   * @param color is new color
   */
  void setColor(Color color)
  {
    color_ = color;
    material_->getTechnique(0)->setAmbient(color_.r_, color_.g_, color_.b_);
    propertyChanged(m_property_color_);
  }

  /**
   * @brief Sets alpha property
   * @param aplha is new alpha
   */
  void setAlpha(float alpha)
  {
    alpha_ = alpha;
    material_->getTechnique(0)->setAmbient(color_.r_, color_.g_, color_.b_);
    propertyChanged(m_property_alpha_);
  }

  /**
   * @brief Sets thickness property
   * @param thickness is new thickness
   */
  void setThickness(float thickness)
  {
    thickness_ = thickness;
    propertyChanged(m_property_thickness_);
  }

  /**
   * @brief Sets show distance property
   * @param show_distance is new show_distance value
   */
  void setShowDistance(bool show_distance)
  {
    show_distance_ = show_distance;
    propertyChanged(m_property_show_distance_);
  }

  /**
   * @brief Sets levels property
   * @param levels is new levels value
   */
  void setLevels(int levels)
  {
    if (levels < 1)
      levels_ = 1;
    else
      levels_ = levels;
    propertyChanged(m_property_levels_);
  }

  /**
   * @brief Sets radius property
   * @param radius is new radius
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
  BoolPropertyWPtr m_property_show_distance_;
  Vector3PropertyWPtr m_property_orientation_;

  // Rotation properties
  Ogre::Vector3 orientation_;

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
#endif /* BUT_DISTANCE_CIRCULAR_INDICATOR_H_ */
