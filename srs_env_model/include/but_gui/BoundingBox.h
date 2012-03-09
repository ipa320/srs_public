/*
 *******************************************************************************
 * $Id: BoundingBox.h 252 2012-02-24 10:54:11Z xlokaj03 $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 24.11.2011
 *******************************************************************************
 */

#include "Primitive.h"

#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_

#define BBOX_MIN_ALPHA 0.0
#define BBOX_MAX_ALPHA 0.1

#define BOUNDING_BOX_CONTROL_NAME "bbox_control"

namespace but_gui
{
/*
 * This class represents Bounding Box primitive
 *
 * Bounding Box allows interaction with the selected object as a movement or rotation.
 * All actions are available and configurable from the menu (right mouse click on the bounding box).
 * With bounding box you can show the object dimensions.
 *
 */
class BoundingBox : public Primitive
{
public:
  /**
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this bounding box
   */
  BoundingBox(InteractiveMarkerServerPtr server, string frame_id, string name);
  /**
   * Constructor.
   */
  BoundingBox()
  {
  }
  /*
   * Inserts bounding box into Interactive marker server
   */
  void insert();

  // Callbacks

  /**
   * Callback for menu
   */
  void menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback);
  /**
   * Callback for interactive markers
   */
  void bboxCallback(const InteractiveMarkerFeedbackConstPtr &feedback);

  //Getters and setters

  /*
   * Gets name of object attached to this bounding box
   */
  string getAttachedObjectName();
  /*
   * Sets name of object attached to this bounding box
   * @param name is name of attached object
   */
  void setAttachedObjectName(string name);

protected:
  Marker bounding_box_, wire_;
  string attached_object_name_;

  void showBoundingBoxControl(bool show);

  void createBoundingBoxControl();
  void create();
  void createMenu();
};

}

#endif /* BOUNDINGBOX_H_ */
