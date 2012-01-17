/*
 *******************************************************************************
 * $Id: Plane.h 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 2.12.2011
 *******************************************************************************
 */

#ifndef PLANE_H_
#define PLANE_H_

#include "GuiObject.h"

namespace but_gui
{

class Plane : public GuiObject
{
public:
  /**
   * Constructor without automatic object creation.
   * @param server_ is Interactive marker server
   * @param frame_id_ is fixed frame
   * @param name_ is name of this plane
   */
  Plane(InteractiveMarkerServerPtr, string, string);
  /**
   * Constructor with automatic object creation.
   * @param server_ is Interactive marker server
   * @param frame_id_ is fixed frame
   * @param name_ is name of this plane
   * @param pose_ is position and orientation of plane
   * @param scale_ is scale is size of this plane
   * @param color_ is RGBA color of this plane
   */
  Plane(InteractiveMarkerServerPtr, string, string, Pose, Scale, ColorRGBA);
  /*
   * Destructor.
   */
  virtual ~Plane();

  /*
   * Create this Plane
   */
  void create();
  /*
   * Insert this Plane into Interactive Marker Server
   */
  void insert();

  // Callbacks

  /**
   * Callback for menu
   */
  void menuCallback(const InteractiveMarkerFeedbackConstPtr &);

private:
  Marker mesh;
  string tag;

  void createMenu();
};

}

#endif /* PLANE_H_ */
