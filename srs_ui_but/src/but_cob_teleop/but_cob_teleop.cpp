/*
 *******************************************************************************
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 9.2.2012
 *******************************************************************************
 */

#include <but_cob_teleop/TeleopCOBMarker.h>

using namespace but_cob_teleop;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "but_gui_cob_teleop");

  TeleopCOBMarker *cobTeleop = new TeleopCOBMarker();

  ROS_INFO("COB Marker Teleop is running...");
  ros::spin();
}
