/*
 *******************************************************************************
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 9.2.2012
 *******************************************************************************
 */

#ifndef TELEOPCOBMARKER_H_
#define TELEOPCOBMARKER_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <but_cob_teleop/interactive_markers_tools.h>

using namespace interactive_markers;
using namespace visualization_msgs;
using namespace std;

namespace but_cob_teleop
{

#define ANGULAR_SCALE 2.2
#define LINEAR_SCALE 1.0

#define NAVIGATION_TRESHOLD 0.2
#define ROTATE_ON_MOVE 0.01
#define ROTATE 2.5

#define MARKER_DRIVER_NAME "marker_driver"
#define MARKER_NAVIGATOR_NAME "marker_navigator"
#define CONTROL_MOVE_NAME "control_move"
#define CONTROL_ROTATE_NAME "control_rotate"
#define CONTROL_NAVIGATION_NAME "controle_naavigation"

typedef boost::shared_ptr<InteractiveMarkerServer> InteractiveMarkerServerPtr;

/*
 * This class handles COB driving using Interactive Markers.
 */
class TeleopCOBMarker
{
public:
  /*
   * Constructor
   */
  TeleopCOBMarker();
  /*
   * Destructor
   */
  virtual ~TeleopCOBMarker()
  {
    server_->erase(MARKER_DRIVER_NAME);
    server_->erase(MARKER_NAVIGATOR_NAME);
    server_->applyChanges();
  }

private:
  /*
   * Markers feedback
   */
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  /*
   * Crates Interactive Markers
   */
  void createMarkers();

  InteractiveMarkerServerPtr server_; // Interactive Marker Server
  ros::Publisher pub_; // Movement publisher
  ros::NodeHandle n_; // Node handler
  geometry_msgs::Pose initial_pose_;

};

}

#endif /* TELEOPCOBMARKER_H_ */
