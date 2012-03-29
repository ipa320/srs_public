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

#include "but_distance_indicator.h"

namespace rviz
{
CButDistanceIndicator::CButDistanceIndicator(const string & name, VisualizationManager * manager) :
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

  // Get scene node
  m_sceneNode_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  m_sceneNode_flipped_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Create basic geometry
  createGeometry();
}

CButDistanceIndicator::~CButDistanceIndicator()
{
  destroyGeometry();
}

bool CButDistanceIndicator::createGeometry()
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

void CButDistanceIndicator::destroyGeometry()
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

void CButDistanceIndicator::onEnable()
{
  m_sceneNode_->setVisible(true);
  m_sceneNode_flipped_->setVisible(true);
}

void CButDistanceIndicator::onDisable()
{
  m_sceneNode_->setVisible(false);
  m_sceneNode_flipped_->setVisible(false);
}

void CButDistanceIndicator::createProperties()
{

  m_property_link_ = property_manager_->createProperty<TFFrameProperty> ("Link", property_prefix_,
                                                                         boost::bind(&CButDistanceIndicator::getLinkString,
                                                                                     this),
                                                                         boost::bind(&CButDistanceIndicator::setLinkString,
                                                                                     this, _1), parent_category_, this);
  setPropertyHelpText(m_property_link_, "Link from which to show radar");

  m_property_color_
      = property_manager_->createProperty<ColorProperty> ("Color", property_prefix_,
                                                          boost::bind(&CButDistanceIndicator::getColor, this),
                                                          boost::bind(&CButDistanceIndicator::setColor, this, _1),
                                                          parent_category_, this);
  setPropertyHelpText(m_property_color_, "Radar color.");

  /*m_property_alpha_
   = property_manager_->createProperty<FloatProperty> ("Alpha", property_prefix_,
   boost::bind(&CButRobotRadar::getAlpha, this),
   boost::bind(&CButRobotRadar::setAlpha, this, _1),
   parent_category_, this);
   setPropertyHelpText(m_property_alpha_, "Alpha channel.");*/

  m_property_levels_
      = property_manager_->createProperty<IntProperty> ("Levels", property_prefix_,
                                                        boost::bind(&CButDistanceIndicator::getLevels, this),
                                                        boost::bind(&CButDistanceIndicator::setLevels, this, _1),
                                                        parent_category_, this);
  setPropertyHelpText(m_property_levels_, "Number of levels.");

  m_property_radius_ = property_manager_->createProperty<FloatProperty> ("Radius", property_prefix_,
                                                                         boost::bind(&CButDistanceIndicator::getRadius, this),
                                                                         boost::bind(&CButDistanceIndicator::setRadius, this,
                                                                                     _1), parent_category_, this);
  setPropertyHelpText(m_property_radius_, "Radar radius (cm).");

  m_property_thickness_ = property_manager_->createProperty<FloatProperty> ("Thickness", property_prefix_,
                                                                            boost::bind(&CButDistanceIndicator::getThickness,
                                                                                        this),
                                                                            boost::bind(&CButDistanceIndicator::setThickness,
                                                                                        this, _1), parent_category_,
                                                                            this);
  setPropertyHelpText(m_property_thickness_, "Circle thickness (cm).");

  m_show_distance_property_
      = property_manager_->createProperty<BoolProperty> ("Show distance", property_prefix_,
                                                         boost::bind(&CButDistanceIndicator::getShowDistance, this),
                                                         boost::bind(&CButDistanceIndicator::setShowDistance, this, _1),
                                                         parent_category_, this);
  setPropertyHelpText(m_show_distance_property_, "Draw text with distance into the scene.");

}

void CButDistanceIndicator::setColor(Color c)
{
  color_ = c;
  propertyChanged(m_property_color_);
  material_->getTechnique(0)->setAmbient(color_.r_, color_.g_, color_.b_);
}

void CButDistanceIndicator::setAlpha(float a)
{
  alpha_ = a;
  propertyChanged(m_property_alpha_);
  material_->getTechnique(0)->setAmbient(color_.r_, color_.g_, color_.b_);
}

void CButDistanceIndicator::setLinkString(string link)
{
  if (isEnabled())
    m_sceneNode_->setVisible(true);

  robot_link_ = link;

  propertyChanged(m_property_link_);
}

void CButDistanceIndicator::update(float wall_dt, float ros_dt)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  Ogre::Quaternion flip_x(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

  // Transform scene node to link position
  if (vis_manager_->getFrameManager()->getTransform(robot_link_, ros::Time(), position, orientation))
  {
    m_sceneNode_->setPosition(position);
    m_sceneNode_->setOrientation(orientation);
    m_sceneNode_->detachAllObjects();

    m_sceneNode_flipped_->setPosition(position);
    m_sceneNode_flipped_->setOrientation(orientation);
    m_sceneNode_flipped_->rotate(flip_x);
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
        /* circle_manual_object_->position(radius_ * cos(theta), 0, radius_ * sin(theta));
         circle_manual_object_->position(radius_ * cos(theta - M_PI / accuracy_), 0, radius_ * sin(theta - M_PI
         / accuracy_));
         circle_manual_object_->position((radius_ - thickness_) * cos(theta - M_PI / accuracy_), 0, (radius_ - thickness_)
         * sin(theta - M_PI / accuracy_));
         circle_manual_object_->position((radius_ - thickness_) * cos(theta), 0, (radius_ - thickness_) * sin(theta));*/
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
      ostringstream radius_str;
      float char_height = radius_ / 2 / 100;
      for (unsigned int level = 1; level <= levels_; level++)
        radius_str << fabs(radius_ * level) << "cm\n";

      ogre_tools::StaticText* text = new ogre_tools::StaticText(radius_str.str());
      text->setCharacterHeight(char_height);
      text->setColor(Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_));
      m_sceneNode_->attachObject(text);

      ogre_tools::StaticText* text_flipped = new ogre_tools::StaticText(radius_str.str());
      text_flipped->setCharacterHeight(char_height);
      text_flipped->setColor(Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_));
      m_sceneNode_flipped_->attachObject(text_flipped);
    }
    setStatus(status_levels::Ok, "Status", "OK");
  }
  else
    setStatus(status_levels::Error, "Status", "Transformation error.");

}
}
