/*
 *******************************************************************************
 * $Id: Plane.h 246 2012-02-23 10:31:43Z xlokaj03 $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 2.12.2011
 *******************************************************************************
 */

#ifndef PLANE_H_
#define PLANE_H_

#include "Primitive.h"

namespace but_gui
{
/*
 * This class represents a Plane primitive.
 *
 * Plane shows only a simple plane which can be tagged as table desk, wall, etc.
 */
class Plane : public Primitive
{
public:
  /**
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name_ is name of this plane
   */
  Plane(InteractiveMarkerServerPtr server, string frame_id, string name);
  /*
   * Insert this Plane into Interactive Marker Server
   */
  void insert();

  // Callbacks

  /**
   * Callback for menu
   */
  void menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback);

private:
  Marker mesh_;
  string tag_;

  void create();
  void createMenu();
};

}

#endif /* PLANE_H_ */
