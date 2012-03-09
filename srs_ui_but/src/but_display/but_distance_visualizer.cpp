/**
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 26.01.2012
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 */

#include "but_distance_visualizer.h"

namespace rviz
{
CButDistanceVisualizer::CButDistanceVisualizer(const string & name, VisualizationManager * manager) :
  Display(name, manager)
{
  // Default properties
  distance_ = 0;
  robot_link_ = DEFAULT_ROBOT_LINK;
  alpha_ = 1.0f;
  color_.r_ = 1.0;
  color_.g_ = 1.0;
  color_.b_ = 0.0;

  // Create a client for the get_closest_point service
  closestPointClient_ = update_nh_.serviceClient<srs_ui_but::GetClosestPoint> (GET_CLOSEST_POINT_SERVICE);

  // Create basic geometry
  createGeometry();
}

CButDistanceVisualizer::~CButDistanceVisualizer()
{
  destroyGeometry();
}

void CButDistanceVisualizer::onEnable()
{
  m_sceneNode_->setVisible(true);
}

void CButDistanceVisualizer::onDisable()
{
  m_sceneNode_->setVisible(false);
}

void CButDistanceVisualizer::createProperties()
{
  m_property_distance_
      = property_manager_->createProperty<StringProperty> ("Distance: ", property_prefix_,
                                                           boost::bind(&CButDistanceVisualizer::getDistance, this),
                                                           boost::bind(&CButDistanceVisualizer::setDistance, this, _1),
                                                           parent_category_);
  setPropertyHelpText(m_property_distance_, "Distance between link and closest surface.");

  rviz::CategoryPropertyWPtr category =
      property_manager_->createCategory("Options", property_prefix_, parent_category_);

  m_property_link_
      = property_manager_->createProperty<TFFrameProperty> ("Link", property_prefix_,
                                                            boost::bind(&CButDistanceVisualizer::getLinkString, this),
                                                            boost::bind(&CButDistanceVisualizer::setLinkString, this,
                                                                        _1), parent_category_, this);
  setPropertyHelpText(m_property_link_, "Link from which to measure distance");

  m_property_color_ = property_manager_->createProperty<ColorProperty> ("Color", property_prefix_,
                                                                        boost::bind(&CButDistanceVisualizer::getColor,
                                                                                    this),
                                                                        boost::bind(&CButDistanceVisualizer::setColor,
                                                                                    this, _1), parent_category_, this);
  setPropertyHelpText(m_property_color_, "Line and text color.");
  m_property_alpha_ = property_manager_->createProperty<FloatProperty> ("Alpha", property_prefix_,
                                                                        boost::bind(&CButDistanceVisualizer::getAlpha,
                                                                                    this),
                                                                        boost::bind(&CButDistanceVisualizer::setAlpha,
                                                                                    this, _1), parent_category_, this);
  setPropertyHelpText(m_property_alpha_, "Alpha channel.");

  m_draw_distance_property_
      = property_manager_->createProperty<BoolProperty> (
                                                         "Draw distance text",
                                                         property_prefix_,
                                                         boost::bind(&CButDistanceVisualizer::getDrawDistance, this),
                                                         boost::bind(&CButDistanceVisualizer::setDrawDistance, this, _1),
                                                         parent_category_, this);
  setPropertyHelpText(m_draw_distance_property_, "Draw text with distance into the scene.");

}

const string CButDistanceVisualizer::getDistance()
{
  ostringstream text_d;
  text_d << fabs(distance_) << " m";
  return text_d.str();
}

const string CButDistanceVisualizer::getLinkString()
{
  return robot_link_;
}

Color CButDistanceVisualizer::getColor()
{
  return color_;
}

float CButDistanceVisualizer::getAlpha()
{
  return alpha_;
}

void CButDistanceVisualizer::setLinkString(string link)
{
  if (isEnabled())
    m_sceneNode_->setVisible(true);

  robot_link_ = link;

  propertyChanged(m_property_link_);
}

void CButDistanceVisualizer::setColor(Color c)
{
  color_ = c;
  propertyChanged(m_property_color_);
}

void CButDistanceVisualizer::setAlpha(float a)
{
  alpha_ = a;
  propertyChanged(m_property_alpha_);
}

bool CButDistanceVisualizer::createGeometry()
{
  // Get scene node
  m_sceneNode_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Create manual object
  static int line_count = 0;
  stringstream ss;
  ss << "line_distance_" << line_count++;
  line_manual_object_ = scene_manager_->createManualObject(ss.str());

  // Set it to dynamic for further update
  line_manual_object_->setDynamic(true);

  // Create basic geometry
  line_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
  line_manual_object_->colour(0.0f, 0.0f, 0.0f, 0.0f);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->position(0, 0, 0);
  line_manual_object_->end();

  // Attach it to the scene
  m_sceneNode_->attachObject(line_manual_object_);

  return true;
}

void CButDistanceVisualizer::destroyGeometry()
{
  // Destroy manual object
  if (line_manual_object_ != 0)
    scene_manager_->destroyManualObject(line_manual_object_);

  // Destroy scene
  if (m_sceneNode_ != 0)
    scene_manager_->destroySceneNode(m_sceneNode_->getName());
}

void CButDistanceVisualizer::update(float wall_dt, float ros_dt)
{
  if (!closestPointClient_.exists())
  {
    setStatus(status_levels::Error, "Service", "get_closest_point service is not available");
    setStatus(status_levels::Error, "Closest point data", "get_closest_point service is not available");
    m_sceneNode_->setVisible(false);
    return;
  }
  else
  {
    setStatus(status_levels::Ok, "Service", "get_closest_point service ready");
    m_sceneNode_->setVisible(true);
  }

  // Set parameters
  closestPointSrv_.request.link = robot_link_;

  // Call service with specified parameters
  closestPointClient_.call(closestPointSrv_);

  // Get data
  pointData_ = closestPointSrv_.response.closest_point_data;

  if (!pointData_.status)
  {
    setStatus(status_levels::Error, "Closest point data", "Cannot get closest point from link " + robot_link_);
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
    line_manual_object_->colour(color_.r_, color_.g_, color_.b_, alpha_);
    line_manual_object_->position(0, 0, 0);
    line_manual_object_->position(pointData_.position.x, pointData_.position.y, pointData_.position.z);
    line_manual_object_->end();
    m_sceneNode_->attachObject(line_manual_object_);

    if (drawDistanceText_)
    {
      ogre_tools::MovableText* text = new ogre_tools::MovableText(getDistance());
      text->setTextAlignment(ogre_tools::MovableText::H_CENTER, ogre_tools::MovableText::V_CENTER);
      text->setCharacterHeight(0.1);
      text->setLocalTranslation(Ogre::Vector3(0.5, 0.5, 0.5));
      text->setColor(Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_));
      m_sceneNode_->attachObject(text);
    }
  }
  setStatus(status_levels::Ok, "Closest point data", "OK");
}
}
