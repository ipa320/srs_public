/******************************************************************************
 * \file
 *
 * $Id: but_cob_stretch_indicator.cpp
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

#include "but_cob_stretch_indicator.h"

#include <srs_ui_but/topics_list.h>

using namespace std;

namespace srs_ui_but
{
CButCOBStretchIndicator::CButCOBStretchIndicator(const string & name, rviz::VisualizationManager * manager) :
  Display(name, manager)
{
  // Default properties
  color_.r_ = 0.0;
  color_.g_ = 0.0;
  color_.b_ = 1.0;
  thickness_ = 0.5;
  correction_ = 0.5;
  accuracy_ = 35;
  alpha_ = 0.3;
  show_distance_ = true;
  show_bbox_ = false;
  style_ = Circle;
  bbox_radius_ = 0.03;

  // Get scene node
  m_sceneNode_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  m_sceneNode_flipped_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Create basic geometry
  createGeometry();

  height_ = 0;
  radius_ = 0;
  timer_ = 0;

  sub_ = update_nh_.subscribe(srs_ui_but::COBStretch_TOPIC, 10, &CButCOBStretchIndicator::stretchCallback, this);
}

CButCOBStretchIndicator::~CButCOBStretchIndicator()
{
  destroyGeometry();
}

bool CButCOBStretchIndicator::createGeometry()
{

  // Create manual object
  static int line_count = 0;
  stringstream ss;
  ss << "cob_stretch_indicator_" << line_count++;

  // Set material
  material_ = Ogre::MaterialManager::getSingleton().create(ss.str(),
                                                           Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->getTechnique(0)->setLightingEnabled(true);
  material_->getTechnique(0)->setAmbient(color_.r_, color_.g_, color_.b_);
  material_->getTechnique(0)->setDiffuse(color_.r_, color_.g_, color_.b_, alpha_);

  material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

  manual_object_ = scene_manager_->createManualObject(ss.str());

  // Set it to dynamic for further update
  manual_object_->setDynamic(true);

  // Attach it to the scene
  m_sceneNode_->attachObject(manual_object_);

  return true;
}

void CButCOBStretchIndicator::destroyGeometry()
{
  // Destroy manual object
  if (manual_object_ != 0)
  {
    scene_manager_->destroyManualObject(manual_object_);
    manual_object_ = NULL;
  }
  // Destroy scene
  if (m_sceneNode_ != 0)
    scene_manager_->destroySceneNode(m_sceneNode_->getName());
  if (m_sceneNode_flipped_ != 0)
    scene_manager_->destroySceneNode(m_sceneNode_flipped_->getName());

}

void CButCOBStretchIndicator::onEnable()
{
  m_sceneNode_->setVisible(true);
  m_sceneNode_flipped_->setVisible(true);
}

void CButCOBStretchIndicator::onDisable()
{
  m_sceneNode_->setVisible(false);
  m_sceneNode_flipped_->setVisible(false);
}

void CButCOBStretchIndicator::createProperties()
{
  rviz::CategoryPropertyWPtr category_rendering = property_manager_->createCategory("Rendering options",
                                                                                    property_prefix_, parent_category_);
  m_property_style_
      = property_manager_->createProperty<rviz::EnumProperty> (
                                                               "Style",
                                                               property_prefix_,
                                                               boost::bind(&CButCOBStretchIndicator::getStyle, this),
                                                               boost::bind(&CButCOBStretchIndicator::setStyle, this, _1),
                                                               category_rendering, this);
  setPropertyHelpText(m_property_style_, "Rendering mode.");
  rviz::EnumPropertyPtr enum_prop = m_property_style_.lock();
  enum_prop->addOption("Circle", Circle);
  enum_prop->addOption("Square", Square);

  m_property_thickness_
      = property_manager_->createProperty<rviz::FloatProperty> ("Thickness", property_prefix_,
                                                                boost::bind(&CButCOBStretchIndicator::getThickness,
                                                                            this),
                                                                boost::bind(&CButCOBStretchIndicator::setThickness,
                                                                            this, _1), category_rendering, this);
  setPropertyHelpText(m_property_thickness_, "Thickness (cm).");

  m_property_correction_
      = property_manager_->createProperty<rviz::FloatProperty> ("Correction", property_prefix_,
                                                                boost::bind(&CButCOBStretchIndicator::getCorrection,
                                                                            this),
                                                                boost::bind(&CButCOBStretchIndicator::setCorrection,
                                                                            this, _1), category_rendering, this);
  setPropertyHelpText(m_property_correction_,
                      "Radius correction due to the difference in thickness of the arm, gripper and fingers. (cm).");

  m_property_show_distance_
      = property_manager_->createProperty<rviz::BoolProperty> ("Show distance", property_prefix_,
                                                               boost::bind(&CButCOBStretchIndicator::getShowDistance,
                                                                           this),
                                                               boost::bind(&CButCOBStretchIndicator::setShowDistance,
                                                                           this, _1), category_rendering, this);
  setPropertyHelpText(m_property_show_distance_, "Draw text with distance into the scene.");

  rviz::CategoryPropertyWPtr category_color = property_manager_->createCategory("Color options", property_prefix_,
                                                                                parent_category_);

  m_property_color_
      = property_manager_->createProperty<rviz::ColorProperty> ("Color", property_prefix_,
                                                                boost::bind(&CButCOBStretchIndicator::getColor, this),
                                                                boost::bind(&CButCOBStretchIndicator::setColor, this,
                                                                            _1), category_color, this);
  setPropertyHelpText(m_property_color_, "Color.");

  m_property_alpha_
      = property_manager_->createProperty<rviz::FloatProperty> ("Alpha", property_prefix_,
                                                                boost::bind(&CButCOBStretchIndicator::getAlpha, this),
                                                                boost::bind(&CButCOBStretchIndicator::setAlpha, this,
                                                                            _1), category_color, this);
  setPropertyHelpText(m_property_alpha_, "Alpha channel.");

  rviz::CategoryPropertyWPtr category_bbox = property_manager_->createCategory("Bounding box", property_prefix_,
                                                                               parent_category_);

  m_property_show_bbox_
      = property_manager_->createProperty<rviz::BoolProperty> (
                                                               "Show bounding box",
                                                               property_prefix_,
                                                               boost::bind(&CButCOBStretchIndicator::getShowBbox, this),
                                                               boost::bind(&CButCOBStretchIndicator::setShowBbox, this,
                                                                           _1), category_bbox, this);
  setPropertyHelpText(m_property_show_bbox_, "Draw bounding box.");

  m_property_bbox_range_
      = property_manager_->createProperty<rviz::FloatProperty> ("Range", property_prefix_,
                                                                boost::bind(&CButCOBStretchIndicator::getBboxRange,
                                                                            this),
                                                                boost::bind(&CButCOBStretchIndicator::setBboxRange,
                                                                            this, _1), category_bbox, this);
  setPropertyHelpText(m_property_bbox_range_, "Range between lines (circles) in the bounding box (cm).");

}

void CButCOBStretchIndicator::update(float wall_dt, float ros_dt)
{
  if (timer_ > 200)
  {
    setStatus(rviz::status_levels::Error, "Status",
              "Cannot get COB's actual stretch. Maybe the cob_stretch_publisher node isn't running.");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  Ogre::Quaternion flip_x(Ogre::Degree(180), Ogre::Vector3::UNIT_X);
  Ogre::Quaternion flip_z(Ogre::Degree(180), Ogre::Vector3::UNIT_Z);

  // Transform scene node to base link position
  if (vis_manager_->getFrameManager()->getTransform(DEFAULT_COB_BASE_LINK, time_stamp_, position, orientation))
  {
    m_sceneNode_->setPosition(position);
    m_sceneNode_->setOrientation(orientation);
    m_sceneNode_->detachAllObjects();

    m_sceneNode_flipped_->setPosition(position);
    m_sceneNode_flipped_->setOrientation(orientation);
    m_sceneNode_flipped_->rotate(flip_x);
    m_sceneNode_flipped_->rotate(flip_z);
    m_sceneNode_flipped_->translate(0, 0, height_);
    m_sceneNode_flipped_->detachAllObjects();

    manual_object_->clear();

    float thickness = thickness_ / 100;
    float correction = correction_ / 100;
    float radius = (radius_ < COB_MIN_WIDTH) ? COB_MIN_WIDTH : radius_;
    radius += correction;

    unsigned point_index = 0;

    switch (style_)
    {
      case Circle:
        manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
        if (show_bbox_)
        {
          float he = (height_ > COB_HEIGHT) ? height_ : COB_HEIGHT;
          for (float h = 0.0f; h < he; h += bbox_radius_/100)
          {
            for (float theta = 0; theta <= 2 * M_PI; theta += M_PI / accuracy_)
            {
              manual_object_->position(radius * cos(theta), radius * sin(theta), h);
              manual_object_->position(radius * cos(theta - M_PI / accuracy_), radius * sin(theta - M_PI / accuracy_),
                                       h);
              manual_object_->position((radius - thickness) * cos(theta - M_PI / accuracy_), (radius - thickness)
                  * sin(theta - M_PI / accuracy_), h);
              manual_object_->position((radius - thickness) * cos(theta), (radius - thickness) * sin(theta), h);
              manual_object_->quad(point_index, point_index + 1, point_index + 2, point_index + 3);
              point_index += 4;
            }
          }
        }
        else
        {
          for (float theta = 0; theta <= 2 * M_PI; theta += M_PI / accuracy_)
          {
            manual_object_->position(radius * cos(theta), radius * sin(theta), height_);
            manual_object_->position(radius * cos(theta - M_PI / accuracy_), radius * sin(theta - M_PI / accuracy_),
                                     height_);
            manual_object_->position((radius - thickness) * cos(theta - M_PI / accuracy_), (radius - thickness)
                * sin(theta - M_PI / accuracy_), height_);
            manual_object_->position((radius - thickness) * cos(theta), (radius - thickness) * sin(theta), height_);
            manual_object_->quad(point_index, point_index + 1, point_index + 2, point_index + 3);
            point_index += 4;
          }
        }
        manual_object_->end();
        break;
      case Square:
        manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_LINE_LIST);
        if (show_bbox_)
        {
          float he = (height_ > COB_HEIGHT) ? height_ : COB_HEIGHT;
          for (float h = 0.0f; h < he; h += bbox_radius_/100)
          {
            manual_object_->position(radius + correction, radius + correction, h);
            manual_object_->position(radius + correction, -radius - correction, h);
            manual_object_->position(radius + correction, -radius - correction, h);
            manual_object_->position(-radius - correction, -radius - correction, h);
            manual_object_->position(-radius - correction, -radius - correction, h);
            manual_object_->position(-radius - correction, radius + correction, h);
            manual_object_->position(-radius - correction, radius + correction, h);
            manual_object_->position(radius + correction, radius + correction, h);
          }
        }
        else
        {
          manual_object_->position(radius + correction, radius + correction, height_);
          manual_object_->position(radius + correction, -radius - correction, height_);
          manual_object_->position(radius + correction, -radius - correction, height_);
          manual_object_->position(-radius - correction, -radius - correction, height_);
          manual_object_->position(-radius - correction, -radius - correction, height_);
          manual_object_->position(-radius - correction, radius + correction, height_);
          manual_object_->position(-radius - correction, radius + correction, height_);
          manual_object_->position(radius + correction, radius + correction, height_);
        }
        manual_object_->end();
        break;
    }
    m_sceneNode_->attachObject(manual_object_);

    if (show_distance_)
    {
      float char_height = 0.1;

      ostringstream radius_str;
      radius_str << fabs(radius * 100) << "cm";
      Ogre::Vector3 trans;
      trans.x = -radius / 2;
      trans.y = radius / 2;
      trans.z = height_;

      ogre_tools::StaticText* text = new ogre_tools::StaticText(radius_str.str());
      text->setCharacterHeight(char_height);
      text->setLocalTranslation(trans);
      text->setColor(Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_));
      m_sceneNode_->attachObject(text);

      ogre_tools::StaticText* text_flipped = new ogre_tools::StaticText(radius_str.str());
      text_flipped->setCharacterHeight(char_height);
      trans.z = 0;
      text_flipped->setLocalTranslation(trans);
      text_flipped->setColor(Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_));
      m_sceneNode_flipped_->attachObject(text_flipped);
    }
    setStatus(rviz::status_levels::Ok, "Status", "OK");
  }
  else
    setStatus(rviz::status_levels::Error, "Status", "Transformation error.");

}
}
