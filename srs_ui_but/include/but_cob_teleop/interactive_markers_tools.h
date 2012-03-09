/*
 *******************************************************************************
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 11.2.2012
 *******************************************************************************
 */

#ifndef INTERACTIVE_MARKERS_TOOLS_H_
#define INTERACTIVE_MARKERS_TOOLS_H_

#include <std_msgs/ColorRGBA.h>
#include <interactive_markers/interactive_marker_server.h>

namespace interactive_markers
{
/*
 * Makes circle from markers and assigns it to specified control
 */
void makeCircle(visualization_msgs::InteractiveMarkerControl &control, float radius, float width,
                std_msgs::ColorRGBA color);
/*
 void makeTarget(visualization_msgs::InteractiveMarkerControl &control, float radius, float width,
 std_msgs::ColorRGBA circle_color, std_msgs::ColorRGBA line_color);*/

}

#endif /* INTERACTIVE_MARKERS_TOOLS_H_ */
