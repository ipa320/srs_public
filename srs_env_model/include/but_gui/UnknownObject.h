/*
 *******************************************************************************
 * $Id: UnknownObject.h 246 2012-02-23 10:31:43Z xlokaj03 $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 2.12.2011
 *******************************************************************************
 */

#ifndef UNKNOWNOBJECT_H_
#define UNKNOWNOBJECT_H_

#include "but_gui/Primitive.h"

namespace but_gui
{
/*
 * This class represents an Unknown Object primitive.
 *
 * Unknown Object shows some obstacles in the scene.
 * Unknown Object can be rotated, translated and scaled.
 */

class UnknownObject : public Primitive
{
public:
  /**
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this object
   */
  UnknownObject(InteractiveMarkerServerPtr server, string frame_id, string name);
  /*
   * Inserts Unknown object into Interactive marker server
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
  void uboxCallback(const InteractiveMarkerFeedbackConstPtr &feedback);

private:
  Marker box_;

  void createUnknownBox();
  void createBox();
  void create();
  void createMenu();
};

}

#endif /* UNKNOWNOBJECT_H_ */
