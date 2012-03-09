/*
 *******************************************************************************
 * $Id: Billboard.h 246 2012-02-23 10:31:43Z xlokaj03 $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 27.11.2011
 *******************************************************************************
 */

#include "Primitive.h"
#include <srs_env_model/BillboardType.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#ifndef BILLBOARD_H_
#define BILLBOARD_H_

using namespace srs_env_model;

namespace but_gui
{
/*
 * This class represents Billboard primitive.
 *
 * Billboard is a simple object created as a plane mesh with texture
 * representing a real world object.
 * Billboard is view facing and can illustrate the movement of the represented object.
 */
class Billboard : public Primitive
{
public:
  /*
   * Constructor.
   * @param server is Interactive marker server
   * @param frame_id is fixed frame
   * @param name is name of this billboard
   */
  Billboard(InteractiveMarkerServerPtr server, string frame_id, string name);
  /*
   * Sets type to the billboard
   * @param type is billboard's type
   */
  void setType(int type);
  /*
   * Gets billboard's type
   */
  int getType();

  /*
   * Inserts this billboard into Interactive marker server
   */
  void insert();
  /**
   * Callback for menu
   */
  void menuCallback(const InteractiveMarkerFeedbackConstPtr &);

private:
  int billboard_type_;
  Marker mesh_;

  void create();
  void createMenu();
  void createMesh();
};
}
#endif /* BILLBOARD_H_ */
