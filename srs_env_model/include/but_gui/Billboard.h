/*
 *******************************************************************************
 * $Id: Billboard.h 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 27.11.2011
 *******************************************************************************
 */

#include "GuiObject.h"
#include <srs_env_model/BillboardType.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#ifndef BILLBOARD_H_
#define BILLBOARD_H_

using namespace srs_env_model;

namespace but_gui
{

class Billboard : public GuiObject
{
public:
  /*
   * Constructor without automatic object creation.
   * @param server_ is Interactive marker server
   * @param frame_id_ is fixed frame
   * @param name_ is name of this billboard
   */
  Billboard(InteractiveMarkerServerPtr, string, string);
  /*
   * Constructor with automatic object creation.
   * @param server_ is Interactive marker server
   * @param frame_id_ is fixed frame
   * @param name_ is name of this billboard
   * @param type_ is type of this billboard
   * @param pose_ is position and orientation of this billboard
   * @param scale_ is scale is size of this billboard
   */
  Billboard(InteractiveMarkerServerPtr, string, string, int, Pose, Scale);
  /*
   * Destructor.
   */
  virtual ~Billboard();

  //Getters and setter

  /*
   * Sets type to the billboard
   * @param type_ is billboard's type
   */
  void setType(int);
  /*
   * Gets billboard's type
   */
  int getType();
  /*
   * Sets direction to the billboard
   * @param direction_ is billboard's direction
   */
  void setDirection(Quaternion);
  /*
   * Gets billboard's direction
   */
  Quaternion getDirection();
  /*
   * Sets velocity to the billboard
   * @param velocity_ is billboard's velocity
   */
  void setVelocity(double);
  /*
   * Gets billboard's velocity
   */
  double getVelocity();

  /*
   * Creates this billboard
   */
  void create();
  /*
   * Inserts this billboard into Interactive marker server
   */
  void insert();

  // Callbacks

  /**
   * Callback for menu
   */
  void menuCallback(const InteractiveMarkerFeedbackConstPtr &);

private:
  int type;
  double velocity;
  Quaternion direction;
  InteractiveMarker object;
  InteractiveMarkerControl trajectoryControl;
  Marker mesh;

  void createMesh();
  void createMenu();
  void createTrajectoryControl();
};
}
#endif /* BILLBOARD_H_ */
