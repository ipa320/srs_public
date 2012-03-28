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

  /**
   * Creates and inserts object into Interactive Marker Server
   */
  void insert();

  /**
   * Sets object's bounding box
   * @param bounding_box_lwh is bounding box length, width and height
   */
  void setBoundingBoxLWH(Point bounding_box_lwh)
  {
    bounding_box_lwh_ = bounding_box_lwh;
  }

  /*
   * Sets position and orientation
   * @param pose is object's position and orientation
   */
  void setPoseLWH(Pose pose, Point bounding_box_lwh);

  /**
   * Callback
   */
  void objectWithBoundingBoxCallback(const InteractiveMarkerFeedbackConstPtr &feedback);
  /**
   * Menu callback
   */
  void menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback);

private:
  void create();
  void createMenu();
  void createMesh();
  void createBoundingBoxControl();

  Point bounding_box_lwh_;
  bool translated_;
};

}

#endif /* OBJECTWITHBOUNDINGBOX_H_ */

