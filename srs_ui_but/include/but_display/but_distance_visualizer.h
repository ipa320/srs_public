/**
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 26.01.2012
 *
 * License: BUT OPEN SOURCE LICENSE
 *
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
#include <ogre_tools/movable_text.h>
#include <srs_ui_but/GetClosestPoint.h>
#include <srs_ui_but/ClosestPoint.h>
#include <time.h>

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
  enum
  {
    UPDATED, READY_FOR_UPDATE
  };

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
   * Subscribe topic
   */
  void subscribe();

  /*
   * Unsubscribe topic
   */
  void unsubscribe();

  /*
   * Get distance property
   */
  const string getDistance();

  /*
   * Get link property
   */
  const string getLinkString();

  /*
   * Get color property
   */
  Color getColor();

  /*
   * Get aplha property
   */
  float getAlpha();

  /*
   * Get draw distance text
   */
  bool getDrawDistance()
  {
    return drawDistanceText_;
  }

  /*
   * Set distance property
   */
  void setDistance(string distance)
  {
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
   * Set draw distance text
   */
  void setDrawDistance(bool draw)
  {
    drawDistanceText_ = draw;
  }

  // Link
  string robot_link_;

  // Link and surface distance
  float distance_;

  // Scene node
  Ogre::SceneNode * m_sceneNode_;

  // Geometry manual object
  Ogre::ManualObject * line_manual_object_;

  // Display properties
  StringPropertyWPtr m_property_distance_;
  TFFramePropertyWPtr m_property_link_;
  FloatPropertyWPtr m_property_alpha_;
  ColorPropertyWPtr m_property_color_;
  BoolPropertyWPtr m_draw_distance_property_;
  IntPropertyWPtr m_refresh_property_;

  // Line color
  Color color_;

  // Line aplha
  float alpha_;

  // Draw distance text
  bool drawDistanceText_;

  // Client for get_closest_point service
  ros::ServiceClient closestPointClient_;

  // GetClosestPoint Service parameters
  srs_ui_but::GetClosestPoint closestPointSrv_;

  // Closest point data
  srs_ui_but::ClosestPoint pointData_;

};
}
#endif /* BUT_DISTANCE_VISUALIZER_H_ */
