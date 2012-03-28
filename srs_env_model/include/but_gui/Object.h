/*
 *******************************************************************************
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 20.1.2012
 *******************************************************************************
 */

#ifndef OBJECT_H_
#define OBJECT_H_

#include "but_gui/Primitive.h"

namespace but_gui
{
/*
 * This class represents detected object.
 */
class Object : public Primitive
{
public:
  /**
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this object
   */
  Object(InteractiveMarkerServerPtr server, string frame_id, string name);
  /*
   * Inserts object into Interactive marker server
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
  void objectCallback(const InteractiveMarkerFeedbackConstPtr &feedback);

protected:
  void createObjectBox();
  void createBox();
  void create();
  void createMenu();

};

}

#endif /* OBJECT_H_ */
