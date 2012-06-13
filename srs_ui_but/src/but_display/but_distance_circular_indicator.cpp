/******************************************************************************
 * \file
 *
 * $Id: but_distance_circular_indicator.cpp 810 2012-05-19 21:47:51Z stancl $
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

#include "but_distance_circular_indicator.h"

#include <srs_ui_but/topics_list.h>

using namespace std;

namespace srs_ui_but
{
CButDistanceCircularIndicator::CButDistanceCircularIndicator(const string & name, rviz::VisualizationManager * manager) :
  Display(name, manager)
{
  // Default properties
  color_.r_ = 0.0;
  color_.g_ = 0.0;
  color_.b_ = 1.0;
  radius_ = 1;
  accuracy_ = 35;
  thickness_ = 0.5;
  robot_link_ = DEFAULT_GRIPPER_LINK;
  levels_ = 3;
  alpha_ = 1.0;
  show_distance_ = true;
  orientation_.x = 0.0;
  orientation_.y = 0.0;
  orientation_.z = 0.0;

  // Get scene node
  m_sceneNode_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  m_sceneNode_flipped_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Create basic geometry
  createGeometry();
}

CButDistanceCircularIndicator::~CButDistanceCircularIndicator()
{
  destroyGeometry();
}

bool CButDistanceCircularIndicator::createGeometry()
{

  // Create manual object
  static int line_count = 0;
  stringstream ss;
  ss << "robot_radar_" << line_count++;

  // Set material
  material_ = Ogre::MaterialManager::getSingleton().create(ss.str(),
                                                           Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->getTechnique(0)->setLightingEnabled(true);
  material_->getTechnique(0)->setAmbient(color_.r_, color_.g_, color_.b_);

  circle_manual_object_ = scene_manager_->createManualObject(ss.str());

  // Set it to dynamic for further update
  circle_manual_object_->setDynamic(true);

  // Create basic geometry
  circle_manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

  circle_manual_object_->end();

  // Attach it to the scene
  m_sceneNode_->attachObject(circle_manual_object_);

  return true;
}

void CButDistanceCircularIndicator::destroyGeometry()
{
  // Destroy manual object
  if (circle_manual_object_ != 0)
  {
    scene_manager_->destroyManualObject(circle_manual_object_);
    circle_manual_object_ = NULL;
  }
  // Destroy scene
  if (m_sceneNode_ != 0)
    scene_manager_->destroySceneNode(m_sceneNode_->getName());
  if (m_sceneNode_flipped_ != 0)
    scene_manager_->destroySceneNode(m_sceneNode_flipped_->getName());

}

void CButDistanceCircularIndicator::onEnable()
{
  m_sceneNode_->setVisible(true);
  m_sceneNode_flipped_->setVisible(true);
}

void CButDistanceCircularIndicator::onDisable()
{
  m_sceneNode_->setVisible(false);
  m_sceneNode_flipped_->setVisible(false);
}

void CButDistanceCircularIndicator::createProperties()
{

  m_property_link_
      = property_manager_->createProperty<rviz::TFFrameProperty> ("Link", property_prefix_,
                                                            boost::bind(&CButDistanceCircularIndicator::getLinkString,
                                                                        this),
                                                            boost::bind(&CButDistanceCircularIndicator::setLinkString,
                                                                        this, _1), parent_category_, this);
  setPropertyHelpText(m_property_link_, "Link from which to show radar");

  m_property_orientation_
      = property_manager_->createProperty<rviz::Vector3Property> ("Orientation", property_prefix_,
                                                            boost::bind(&CButDistanceCircularIndicator::getOrientation,
                                                                        this),
                                                            boost::bind(&CButDistanceCircularIndicator::setOrientation,
                                                                        this, _1), parent_category_, this);
  setPropertyHelpText(m_property_orientation_, "Radar orientation.");

  m_property_color_
      = property_manager_->createProperty<rviz::ColorProperty> ("Color", property_prefix_,
                                                          boost::bind(&CButDistanceCircularIndicator::getColor, this),
                                                          boost::bind(&CButDistanceCircularIndicator::setColor, this,
                                                                      _1), parent_category_, this);
  setPropertyHelpText(m_property_color_, "Radar color.");

  /*m_property_alpha_
   = property_manager_->createProperty<FloatProperty> ("Alpha", property_prefix_,
   boost::bind(&CButRobotRadar::getAlpha, this),
   boost::bind(&CButRobotRadar::setAlpha, this, _1),
   parent_category_, this);
   setPropertyHelpText(m_property_alpha_, "Alpha channel.");*/

  m_property_levels_
      = property_manager_->createProperty<rviz::IntProperty> (
                                                        "Levels",
                                                        property_prefix_,
                                                        boost::bind(&CButDistanceCircularIndicator::getLevels, this),
                                                        boost::bind(&CButDistanceCircularIndicator::setLevels, this, _1),
                                                        parent_category_, this);
  setPropertyHelpText(m_property_levels_, "Number of levels.");

  m_property_radius_
      = property_manager_->createProperty<rviz::FloatProperty> ("Radius", property_prefix_,
                                                          boost::bind(&CButDistanceCircularIndicator::getRadius, this),
                                                          boost::bind(&CButDistanceCircularIndicator::setRadius, this,
                                                                      _1), parent_category_, this);
  setPropertyHelpText(m_property_radius_, "Radar radius (cm).");

  m_property_thickness_
      = property_manager_->createProperty<rviz::FloatProperty> ("Thickness", property_prefix_,
                                                          boost::bind(&CButDistanceCircularIndicator::getThickness,
                                                                      this),
                                                          boost::bind(&CButDistanceCircularIndicator::setThickness,
                                                                      this, _1), parent_category_, this);
  setPropertyHelpText(m_property_thickness_, "Circle thickness (cm).");

  m_property_show_distance_
      = property_manager_->createProperty<rviz::BoolProperty> ("Show distance", property_prefix_,
                                                         boost::bind(&CButDistanceCircularIndicator::getShowDistance,
                                                                     this),
                                                         boost::bind(&CButDistanceCircularIndicator::setShowDistance,
                                                                     this, _1), parent_category_, this);
  setPropertyHelpText(m_property_show_distance_, "Draw text with distance into the scene.");

}

void CButDistanceCircularIndicator::update(float wall_dt, float ros_dt)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  Ogre::Quaternion flip_x(Ogre::Degree(180), Ogre::Vector3::UNIT_X);
  Ogre::Quaternion flip_z(Ogre::Degree(180), Ogre::Vector3::UNIT_Z);

  Ogre::Quaternion rotate_x(Ogre::Degree(orientation_.x), Ogre::Vector3::UNIT_X);
  Ogre::Quaternion rotate_y(Ogre::Degree(orientation_.y), Ogre::Vector3::UNIT_Y);
  Ogre::Quaternion rotate_z(Ogre::Degree(orientation_.z), Ogre::Vector3::UNIT_Z);

  // Transform scene node to link position
  if (vis_manager_->getFrameManager()->getTransform(robot_link_, ros::Time(), position, orientation))
  {
    m_sceneNode_->setPosition(position);
    m_sceneNode_->setOrientation(orientation);
    m_sceneNode_->rotate(rotate_x);
    m_sceneNode_->rotate(rotate_y);
    m_sceneNode_->rotate(rotate_z);
    m_sceneNode_->detachAllObjects();

    m_sceneNode_flipped_->setPosition(position);
    m_sceneNode_flipped_->setOrientation(orientation);
    m_sceneNode_flipped_->rotate(rotate_x);
    m_sceneNode_flipped_->rotate(rotate_y);
    m_sceneNode_flipped_->rotate(rotate_z);
    m_sceneNode_flipped_->rotate(flip_x);
    m_sceneNode_flipped_->rotate(flip_z);
    m_sceneNode_flipped_->detachAllObjects();

    circle_manual_object_->clear();

    circle_manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    unsigned point_index = 0;
    float thickness = thickness_ / 100;

    for (unsigned int level = 1; level <= levels_; level++)
    {
      float radius = radius_ * level / 100;
      for (float theta = 0; theta <= 2 * M_PI; theta += M_PI / accuracy_)
      {
        circle_manual_object_->position(radius * cos(theta), radius * sin(theta), 0);
        circle_manual_object_->position(radius * cos(theta - M_PI / accuracy_), radius * sin(theta - M_PI / accuracy_),
                                        0);
        circle_manual_object_->position((radius - thickness) * cos(theta - M_PI / accuracy_), (radius - thickness)
            * sin(theta - M_PI / accuracy_), 0);
        circle_manual_object_->position((radius - thickness) * cos(theta), (radius - thickness) * sin(theta), 0);
        circle_manual_object_->quad(point_index, point_index + 1, point_index + 2, point_index + 3);
        point_index += 4;
      }
    }
    circle_manual_object_->end();
    m_sceneNode_->attachObject(circle_manual_object_);

    if (show_distance_)
    {
      float char_height = radius_ / 300;

      if (char_height > 0.1)
        char_height = 0.1;

      for (unsigned int level = 1; level <= levels_; level++)
      {
        float radius = radius_ * level;
        ostringstream radius_str;
        radius_str << fabs(radius) << "cm";
        Ogre::Vector3 trans;
        trans.x = -char_height / 2 * radius_str.str().length();
        trans.y = radius / 100 - char_height / 2;
        trans.z = 0.0f;

        ogre_tools::StaticText* text = new ogre_tools::StaticText(radius_str.str());
        text->setCharacterHeight(char_height);
        text->setLocalTranslation(trans);
        text->setColor(Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_));
        m_sceneNode_->attachObject(text);

        ogre_tools::StaticText* text_flipped = new ogre_tools::StaticText(radius_str.str());
        text_flipped->setCharacterHeight(char_height);
        text_flipped->setLocalTranslation(trans);
        text_flipped->setColor(Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_));
        m_sceneNode_flipped_->attachObject(text_flipped);
      }
    }
    setStatus(rviz::status_levels::Ok, "Status", "OK");
  }
  else
    setStatus(rviz::status_levels::Error, "Status", "Transformation error.");

}
}
