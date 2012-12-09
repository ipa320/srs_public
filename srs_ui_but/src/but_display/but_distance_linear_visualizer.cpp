/******************************************************************************
 * \file
 *
 * $Id: but_distance_linear_visualizer.cpp 810 2012-05-19 21:47:51Z stancl $
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

#include "but_distance_linear_visualizer.h"

#include <srs_ui_but/topics_list.h>
#include <srs_ui_but/services_list.h>

using namespace std;

namespace srs_ui_but
{
CButDistanceLinearVisualizer::CButDistanceLinearVisualizer(const string & name, rviz::VisualizationManager * manager) :
    Display(name, manager)
{
  // Default properties
  distance_ = 0.0;
  robot_link_ = DEFAULT_ROBOT_LINK;
  alpha_ = 1.0f;
  color_.r_ = 1.0;
  color_.g_ = 1.0;
  color_.b_ = 0.0;
  thickness_ = 0.01;
  show_distance_ = true;

  // Create a client for the get_closest_point service
  getClosestPointClient_ = update_nh_.serviceClient<srs_ui_but::GetClosestPoint>(GetClosestPoint_SRV);
  setPointCloudTopicClient_ = update_nh_.serviceClient<srs_ui_but::SetPointCloudTopic>(SetPointCloudTopic_SRV);

  // Create basic geometry
  createGeometry();
}

CButDistanceLinearVisualizer::~CButDistanceLinearVisualizer()
{
  destroyGeometry();
}

bool CButDistanceLinearVisualizer::createGeometry()
{
  // Get scene node
  m_sceneNode_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Create manual object
  static int line_count = 0;
  stringstream ss;
  ss << "line_distance_" << line_count++;

  // Set material
  material_ = Ogre::MaterialManager::getSingleton().create(ss.str(),
                                                           Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->getTechnique(0)->setLightingEnabled(true);
  material_->getTechnique(0)->setAmbient(color_.r_, color_.g_, color_.b_);
  material_->getTechnique(0)->setDiffuse(color_.r_, color_.g_, color_.b_, alpha_);
  material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

  line_manual_object_ = scene_manager_->createManualObject(ss.str());

  // Set it to dynamic for further update
  line_manual_object_->setDynamic(true);

  // Create basic geometry
  line_manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  //line_manual_object_->colour(0.0f, 0.0f, 0.0f, 0.0f);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->end();

  // Attach it to the scene
  m_sceneNode_->attachObject(line_manual_object_);

  m_sceneNode_->setVisible(false);

  return true;
}

void CButDistanceLinearVisualizer::destroyGeometry()
{
  // Destroy manual object
  if (line_manual_object_ != 0)
  {
    scene_manager_->destroyManualObject(line_manual_object_);
    line_manual_object_ = NULL;
  }
  // Destroy scene
  if (m_sceneNode_ != 0)
    scene_manager_->destroySceneNode(m_sceneNode_->getName());
}

void CButDistanceLinearVisualizer::onEnable()
{
  if (m_sceneNode_ != 0)
    m_sceneNode_->setVisible(true);
}

void CButDistanceLinearVisualizer::onDisable()
{
  if (m_sceneNode_ != 0)
    m_sceneNode_->setVisible(false);
}

void CButDistanceLinearVisualizer::createProperties()
{
  m_property_distance_ = property_manager_->createProperty<rviz::StringProperty>(
      "Distance: ", property_prefix_, boost::bind(&CButDistanceLinearVisualizer::getDistance, this),
      rviz::StringProperty::Setter(), parent_category_);
  setPropertyHelpText(m_property_distance_, "Distance between link and closest surface.");

  rviz::CategoryPropertyWPtr category = property_manager_->createCategory("Options", property_prefix_,
                                                                          parent_category_);

  // Add subscribed topic property
  m_property_topic_ = property_manager_->createProperty<rviz::ROSTopicStringProperty>(
      "Topic", property_prefix_, boost::bind(&CButDistanceLinearVisualizer::getTopic, this),
      boost::bind(&CButDistanceLinearVisualizer::setTopic, this, _1), category, this);
  // Add helper text
  setPropertyHelpText(m_property_topic_, "sensor_msgs::PointCloud2 topic to subscribe to.");

  rviz::ROSTopicStringPropertyPtr topic_prop = m_property_topic_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::PointCloud2>());

  m_property_link_ = property_manager_->createProperty<rviz::TFFrameProperty>(
      "Link", property_prefix_, boost::bind(&CButDistanceLinearVisualizer::getLinkString, this),
      boost::bind(&CButDistanceLinearVisualizer::setLinkString, this, _1), category, this);
  setPropertyHelpText(m_property_link_, "Link from which to measure distance");

  m_property_color_ = property_manager_->createProperty<rviz::ColorProperty>(
      "Color", property_prefix_, boost::bind(&CButDistanceLinearVisualizer::getColor, this),
      boost::bind(&CButDistanceLinearVisualizer::setColor, this, _1), category, this);
  setPropertyHelpText(m_property_color_, "Line and text color.");

  m_property_alpha_ = property_manager_->createProperty<rviz::FloatProperty>(
      "Alpha", property_prefix_, boost::bind(&CButDistanceLinearVisualizer::getAlpha, this),
      boost::bind(&CButDistanceLinearVisualizer::setAlpha, this, _1), category, this);
  setPropertyHelpText(m_property_alpha_, "Alpha channel.");

  m_property_thickness_ = property_manager_->createProperty<rviz::FloatProperty>(
      "Line Thickness", property_prefix_, boost::bind(&CButDistanceLinearVisualizer::getThickness, this),
      boost::bind(&CButDistanceLinearVisualizer::setThickness, this, _1), category, this);
  setPropertyHelpText(m_property_thickness_, "Line thickness (maximum value is 0,5).");

  m_show_distance_property_ = property_manager_->createProperty<rviz::BoolProperty>(
      "Show distance", property_prefix_, boost::bind(&CButDistanceLinearVisualizer::getShowDistance, this),
      boost::bind(&CButDistanceLinearVisualizer::setShowDistance, this, _1), category, this);
  setPropertyHelpText(m_show_distance_property_, "Draw text with distance into the scene.");

}

void CButDistanceLinearVisualizer::update(float wall_dt, float ros_dt)
{
  if (!getClosestPointClient_.exists())
  {
    setStatus(rviz::status_levels::Error, "Service", "get_closest_point service is not available");
    setStatus(rviz::status_levels::Error, "Closest point data", "get_closest_point service is not available");
    m_sceneNode_->setVisible(false);
    return;
  }
  else
  {
    setStatus(rviz::status_levels::Ok, "Service", "get_closest_point service ready");
    m_sceneNode_->setVisible(true);
  }

  // Set parameters
  getClosestPointSrv_.request.link = robot_link_;

  // Call service with specified parameters
  getClosestPointClient_.call(getClosestPointSrv_);

  // Get data
  pointData_ = getClosestPointSrv_.response.closest_point_data;

  if (!pointData_.status)
  {
    setStatus(rviz::status_levels::Error, "Closest point data", "Cannot get closest point from link " + robot_link_);
    m_sceneNode_->setVisible(false);
    return;
  }
  if (getClosestPointSrv_.response.topic != "")
  {
    pc_topic_ = getClosestPointSrv_.response.topic;
    propertyChanged(m_property_topic_);
    setStatus(rviz::status_levels::Ok, "Point Cloud Topic", pc_topic_);
    m_sceneNode_->setVisible(true);
  }
  else
  {
    setStatus(rviz::status_levels::Error, "Point Cloud Topic", "not specified");
    m_sceneNode_->setVisible(false);
    return;
  }

  distance_ = pointData_.distance;
  propertyChanged(m_property_distance_);

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  // Transform scene node to link position
  if (vis_manager_->getFrameManager()->getTransform(robot_link_, pointData_.time_stamp, position, orientation))
  {
    m_sceneNode_->setPosition(position);
    m_sceneNode_->setOrientation(orientation);
    m_sceneNode_->detachAllObjects();

    line_manual_object_->beginUpdate(0);
    //line_manual_object_->colour(color_.r_, color_.g_, color_.b_, alpha_);
    line_manual_object_->setMaterialName(0, material_->getName());
    line_manual_object_->position(-thickness_, 0, 0);
    line_manual_object_->position(thickness_, 0, 0);
    line_manual_object_->position(pointData_.position.x - thickness_, pointData_.position.y, pointData_.position.z);
    line_manual_object_->position(thickness_, 0, 0);
    line_manual_object_->position(pointData_.position.x - thickness_, pointData_.position.y, pointData_.position.z);
    line_manual_object_->position(pointData_.position.x + thickness_, pointData_.position.y, pointData_.position.z);
    line_manual_object_->position(0, 0, -thickness_);
    line_manual_object_->position(0, 0, thickness_);
    line_manual_object_->position(pointData_.position.x, pointData_.position.y, pointData_.position.z - thickness_);
    line_manual_object_->position(0, 0, thickness_);
    line_manual_object_->position(pointData_.position.x, pointData_.position.y, pointData_.position.z - thickness_);
    line_manual_object_->position(pointData_.position.x, pointData_.position.y, pointData_.position.z + thickness_);

    line_manual_object_->end();
    m_sceneNode_->attachObject(line_manual_object_);

    if (show_distance_)
    {
      ogre_tools::MovableText* text = new ogre_tools::MovableText(getDistance());
      text->setTextAlignment(ogre_tools::MovableText::H_CENTER, ogre_tools::MovableText::V_CENTER);
      text->setCharacterHeight(0.1);
      text->setLocalTranslation(Ogre::Vector3(0.25, 0.25, 0));
//      text->setLocalTranslation(Ogre::Vector3(pointData_.position.x, pointData_.position.y, pointData_.position.z));
      text->setColor(Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_));
      m_sceneNode_->attachObject(text);
    }
  }
  setStatus(rviz::status_levels::Ok, "Closest point data", "OK");
}
}
