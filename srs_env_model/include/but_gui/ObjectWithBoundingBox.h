/*
 *******************************************************************************
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 24.2.2012
 *******************************************************************************
 */

#ifndef OBJECTWITHBOUNDINGBOX_H_
#define OBJECTWITHBOUNDINGBOX_H_

#include <but_gui/BoundingBox.h>

namespace but_gui
{

class ObjectWithBoundingBox : public BoundingBox
{
public:
  /**
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this object
   */
  ObjectWithBoundingBox(InteractiveMarkerServerPtr server, string frame_id, string name);

  void insert();

  void setBoundingBoxMin(Point bounding_box_min)
  {
    bounding_box_min_ = bounding_box_min;
  }
  void setBoundingBoxMax(Point bounding_box_max)
  {
    bounding_box_max_ = bounding_box_max;
  }

  void objectWithBoundingBoxCallback(const InteractiveMarkerFeedbackConstPtr &feedback);
  void menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback);

private:
  void create();
  void createMenu();
  void createMesh();
  void createBoundingBoxControl();

  Point bounding_box_min_, bounding_box_max_;
};

}

#endif /* OBJECTWITHBOUNDINGBOX_H_ */

