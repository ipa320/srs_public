/*
 *******************************************************************************
 * $Id: UnknownObject.h 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 2.12.2011
 *******************************************************************************
 */

#ifndef UNKNOWNOBJECT_H_
#define UNKNOWNOBJECT_H_

#include "but_gui/GuiObject.h"

namespace but_gui
{

class UnknownObject : public GuiObject
{
public:
  /**
   * Constructor without automatic object creation.
   * @param server_ is Interactive marker server
   * @param frame_id_ is fixed frame
   * @param name_ is name of this object
   */
  UnknownObject(InteractiveMarkerServerPtr, string, string);
  /**
   * Constructor with automatic object creation.
   * @param server_ is Interactive marker server
   * @param frame_id_ is fixed frame
   * @param name_ is name of this bounding box
   * @param pose_ is position and orientation of this bounding box
   * @param scale_ is scale is size of this bounding box
   * @param description_ is description of this bounding box
   */
  UnknownObject(InteractiveMarkerServerPtr, string, string, Pose, Scale, string);
  /*
   * Destructor.
   */
  virtual ~UnknownObject();

  /*
   * Creates this Unknown object
   */
  void create();
  /*
   * Inserts Unknown object into Interactive marker server
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
  void uboxCallback(const InteractiveMarkerFeedbackConstPtr &);

private:
  Marker box;
  bool immediateInteraction;

  void createUnknownBox();
  void createBox();
  void createMenu();
};

}

#endif /* UNKNOWNOBJECT_H_ */
