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

#ifndef BUT_DISTANCE_VISUALIZER_H_
#define BUT_DISTANCE_VISUALIZER_H_

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
#include <srs_ui_but/GetClosestPoint.h>
#include <srs_ui_but/ClosestPoint.h>
#include <time.h>
#include <GL/gl.h>

#define DEFAULT_ROBOT_LINK "/sdh_palm_link"
#define GET_CLOSEST_POINT_SERVICE "but_services/get_closest_point"

using namespace std;

namespace rviz
{

/*
 * This tool displays closest point from oa selected robot's link
 */
class CButDistanceVisualizer : public rviz::Display

{
public:
  /*
   * Constructor
   */
  CButDistanceVisualizer(const string & name, rviz::VisualizationManager * manager);

  /*
   * Destructor
   */
  virtual ~CButDistanceVisualizer();

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
   * Get distance property
   */
  const string getDistance();

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
   * Get draw distance text
   */
  bool getShowDistance()
  {
    return showDistance_;
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
    if (thickness > 0.1)
      thickness_ = 0.1;
    else
      thickness_ = thickness;

    propertyChanged(m_property_thickness_);
  }

  /*
   * Set draw distance text
   */
  void setShowDistance(bool show)
  {
    showDistance_ = show;
    propertyChanged(m_show_distance_property_);
  }

  // Link
  string robot_link_;

  // Link and surface distance
  float distance_;

  // Scene node
  Ogre::SceneNode * m_sceneNode_;

  // Geometry manual object
  Ogre::ManualObject * line_manual_object_;

  // Material
  Ogre::MaterialPtr material_;

  // Display properties
  StringPropertyWPtr m_property_distance_;
  TFFramePropertyWPtr m_property_link_;
  FloatPropertyWPtr m_property_alpha_, m_property_thickness_;
  ColorPropertyWPtr m_property_color_;
  BoolPropertyWPtr m_show_distance_property_;
  IntPropertyWPtr m_refresh_property_;

  // Line color
  Color color_;

  // Line aplha
  float alpha_;

  // Line thickness
  float thickness_;

  // Draw distance text
  bool showDistance_;

  // Client for get_closest_point service
  ros::ServiceClient closestPointClient_;

  // GetClosestPoint Service parameters
  srs_ui_but::GetClosestPoint closestPointSrv_;

  // Closest point data
  srs_ui_but::ClosestPoint pointData_;

};
}
#endif /* BUT_DISTANCE_VISUALIZER_H_ */
