/******************************************************************************
 * \file
 *
 * $Id:
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 07/02/2012
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
#pragma once
#ifndef BUT_COB_STRETCH_INDICATOR_H_
#define BUT_COB_STRETCH_INDICATOR_H_

#include <ros/ros.h>
#include <ros/node_handle.h>

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

#include <srs_ui_but/but_ogre_tools/static_text.h>
#include <srs_ui_but/COBStretch.h>

#define COB_HEIGHT 1.4
#define COB_MIN_WIDTH 0.444

namespace srs_ui_but
{

/**
 * @brief This tool displays distances around specified robot's link.
 *
 * @author Tomas Lokaj
 */
class CButCOBStretchIndicator : public rviz::Display

{
public:
  /**
   * @brief Constructor
   */
  CButCOBStretchIndicator(const std::string & name, rviz::VisualizationManager * manager);

  /**
   * @brief Destructor
   */
  virtual ~CButCOBStretchIndicator();

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
   * @brief Gets color property
   * @return color
   */
  rviz::Color getColor()
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
   * @brief Gets bbox radius property
   * @return radius
   */
  float getBboxRange()
  {
    return bbox_radius_;
  }

  /**
   * @brief Gets correction property
   * @return correction
   */
  float getCorrection()
  {
    return correction_;
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
   * @brief Gets show bbox property
   * @return show_bbox
   */
  bool getShowBbox()
  {
    return show_bbox_;
  }

  /**
   * @brief Gets style property
   * @return style
   */
  int getStyle()
  {
    return style_;
  }

  /**
   * @brief Sets color property
   * @param color is new color
   */
  void setColor(rviz::Color color)
  {
    color_ = color;
    material_->getTechnique(0)->setAmbient(color_.r_, color_.g_, color_.b_);
    material_->getTechnique(0)->setDiffuse(color_.r_, color_.g_, color_.b_, alpha_);
    propertyChanged(m_property_color_);
  }

  /**
   * @brief Sets alpha property
   * @param aplha is new alpha
   */
  void setAlpha(float alpha)
  {
    alpha_ = alpha;
    //material_->getTechnique(0)->setAmbient(color_.r_, color_.g_, color_.b_);
    material_->getTechnique(0)->setDiffuse(color_.r_, color_.g_, color_.b_, alpha_);
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
   * @brief Sets bbox radius property
   * @param radius is new bbox radius
   */
  void setBboxRange(float radius)
  {
    bbox_radius_ = radius;
    propertyChanged(m_property_bbox_range_);
  }

  /**
   * @brief Sets correction property
   * @param correction is new correction
   */
  void setCorrection(float correction)
  {
    correction_ = correction;
    propertyChanged(m_property_correction_);
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
   * @brief Sets show bbox property
   * @param show_bbox is new show_bbox value
   */
  void setShowBbox(bool show_bbox)
  {
    show_bbox_ = show_bbox;
    propertyChanged(m_property_show_bbox_);
  }

  /**
   * @brief Sets style property
   * @param style is new style
   */
  void setStyle(int style)
  {
    ROS_ASSERT( style < StyleCount );
    style_ = style;
    propertyChanged(m_property_style_);
  }

  /**
   * Callback function for stretch update
   * @param update
   */
  void stretchCallback(const srs_ui_but::COBStretchConstPtr &update)
  {
    radius_ = update->radius;
    height_ = update->height;
    time_stamp_ = update->time_stamp;
    timer_=0;
  }

  // Scene node
  Ogre::SceneNode * m_sceneNode_, *m_sceneNode_flipped_;

  // Geometry manual object
  Ogre::ManualObject * manual_object_;

  // Material
  Ogre::MaterialPtr material_;

  // Display properties
  rviz::ColorPropertyWPtr m_property_color_;
  rviz::EnumPropertyWPtr m_property_style_;
  rviz::FloatPropertyWPtr m_property_alpha_, m_property_correction_, m_property_thickness_, m_property_bbox_range_;
  rviz::BoolPropertyWPtr m_property_show_distance_, m_property_show_bbox_;

  /*m_property_thickness_, m_property_radius_;
   rviz::IntPropertyWPtr m_property_levels_;
   rviz::Vector3PropertyWPtr m_property_orientation_;*/

  // Properties
  rviz::Color color_;
  float thickness_, accuracy_, alpha_, correction_, bbox_radius_;
  bool show_distance_, show_bbox_;
  int style_;

  /**
   * \enum Style
   * \brief The different styles
   */
  enum Style
  {
    Circle, Square, StyleCount,
  };

  // Stretch properties
  float height_, radius_;
  ros::Time time_stamp_;

  // Strech subscriber
  ros::Subscriber sub_;

  int timer_;
};

} // namespace srs_ui_but

#endif /* BUT_COB_STRETCH_INDICATOR_H_ */
