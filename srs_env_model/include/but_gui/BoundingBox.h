/*
 *******************************************************************************
 * $Id: BoundingBox.h 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 24.11.2011
 *******************************************************************************
 */

#include "GuiObject.h"

#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_

#define BBOX_MIN_ALPHA 0.0
#define BBOX_MAX_ALPHA 0.1

namespace but_gui
{

class BoundingBox : public GuiObject
{
public:
  /**
   * Constructor without automatic object creation.
   * @param server_ is Interactive marker server
   * @param frame_id_ is fixed frame
   * @param name_ is name of this bounding box
   */
  BoundingBox(InteractiveMarkerServerPtr, string, string);
  /**
   * Constructor with automatic object creation.
   * @param server_ is Interactive marker server
   * @param frame_id_ is fixed frame
   * @param name_ is name of this bounding box
   * @param objectName_ is name of object controlled by this bounding box
   * @param pose_ is position and orientation of this bounding box
   * @param scale_ is scale is size of this bounding box
   * @param color_ is RGBA color of this bounding box
   * @param description_ is description of this bounding box
   */
  BoundingBox(InteractiveMarkerServerPtr, string, string, string, Pose, Scale, ColorRGBA, string);
  /*
   * Destructor.
   */
  virtual ~BoundingBox();

  /*
   * Creates this bounding box
   */
  void create();
  /*
   * Inserts bounding box into Interactive marker server
   */
  void insert();

  // Callbacks

  /**
   * Callback for menu
   */
  void menuCallback(const InteractiveMarkerFeedbackConstPtr &);
  /**
   * Callback for interactive markers
   */
  void bboxCallback(const InteractiveMarkerFeedbackConstPtr &);

  //Getters and setters

  /*
   * Gets name of object attached to this bounding box
   */
  string getAttachedObjectName();
  /*
   * Sets name of object attached to this bounding box
   * @param name_ is name of attached object
   */
  void setAttachedObjectName(string);

private:
  Marker box;
  bool immediateInteraction;
  string attachedObjectName;

  void createBoundingBox();
  void createBox();
  void createMenu();
};

}

#endif /* BOUNDINGBOX_H_ */
