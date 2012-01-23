/*
 *******************************************************************************
 * $Id: GuiObject.h 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 24.11.2011
 *******************************************************************************
 */

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/tools.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#define MEASURE_TEXT_SIZE 0.2

using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace std_msgs;
using namespace interactive_markers;
using namespace boost;
using namespace std;

#ifndef CBUTOBJECT_H_
#define CBUTOBJECT_H_

namespace but_gui
{

typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;
typedef Vector3 Scale;

/*
 * Returns maximal scale value
 * @param scale is vector of scale values
 */
float maxScale(Scale);

class GuiObject
{
public:
  virtual void create()
  /*
   * Create Object
   */
  {
  }
  virtual void createMenu()
  /*
   * Create Menu
   */
  {
  }
  /*
   * Insert object into Interactive Marker Server
   */
  virtual void insert();
  /*
   * Erase object from Interactive Marker Server
   */
  void erase();
  /*
   * Change object's color
   */
  void changeColor(ColorRGBA);

  //Getters and setters

  /*
   * Gets object's name
   */
  string getName();
  /*
   * Sets color to object
   * @param color_ is object's color
   */
  void setColor(ColorRGBA);
  /*
   * Gets object's color
   */
  ColorRGBA getColor();
  /*
   * Sets position and orientation to object
   * @param pose_ is object's position and orientation
   */
  void setPose(Pose);
  /*
   * Gets object's position and orientation
   */
  Pose getPose();
  /*
   * Sets scale to object
   * @param scale_ is object's scale
   */
  void setScale(Scale);
  /*
   * Gets object's scale
   */
  Scale getScale();
  /*
   * Sets description to object
   * @param description_ is object's description
   */
  void setDescription(string);
  /*
   * Gets object's description
   */
  string getDescription();
  /*
   * Sets fixed frame to object
   * @param frame_id_ is object's fixed frame
   */
  void setFrameID(string);
  /*
   * Gets object's fixed frame
   */
  string getFrameID();

protected:
  /*
   * Creates controls
   */
  void createControls();
  /*
   * Creates measure controls
   */
  void createMeasureControl();
  /*
   * Creates description controls
   */
  void createDescriptionControl();
  /*
   * Adds movement controls
   * @param marker is Interactive marker
   */
  void addMovementControls(InteractiveMarker &);
  /*
   * Adds rotation controls
   * @param marker is Interactive marker
   */
  void addRotationControls(InteractiveMarker &);
  /*
   * Removes movement controls
   * @param marker is Interactive marker
   * @param rotation specifies if rotation controls should remain
   */
  void removeMovementControls(InteractiveMarker &, bool);
  /*
   * Removes rotation controls
   * @param marker is Interactive marker
   * @param rotation specifies if movement controls should remain
   */
  void removeRotationControls(InteractiveMarker &, bool);

  InteractiveMarker object;
  InteractiveMarkerControl control, descriptionControl, measureControl;
  InteractiveMarkerControl cMoveX, cMoveY, cMoveZ, cRotateX, cRotateY, cRotateZ;
  string name, description, frame_id;
  InteractiveMarkerServerPtr server;
  MenuHandler menu_handler;
  Pose pose;
  Scale scale;
  ColorRGBA color;
  int baseControlCount;
};
}
#endif /* CBUTOBJECT_H_ */
