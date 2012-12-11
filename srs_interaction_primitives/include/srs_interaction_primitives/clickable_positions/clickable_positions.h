/******************************************************************************
 * \file
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 12/09/2012
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CLICKABLE_POSITIONS_H_
#define CLICKABLE_POSITIONS_H_

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <interactive_markers/interactive_marker_server.h>
#include <srs_interaction_primitives/topics_list.h>

namespace srs_interaction_primitives
{
typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;

/**
 * @brief This class represents clickable positions, which are visualized by the Interactive
 * Markers as a sphere. Clicked position is published at the specified topic.
 *
 * @author Tomas Lokaj
 */
class ClickablePositionsMarker
{
public:
  /**
   * @brief Constructor.
   * @param imServer is Interactive Marker Server
   * @param frame_id Fixed Frame
   * @param topic Topic suffix for publishing clicked position
   * @param radius Sphere radius
   * @param color Sphere color
   * @param positions Clickable positions
   */
  ClickablePositionsMarker(InteractiveMarkerServerPtr imServer,std::string frame_id, std::string topic, float radius, std_msgs::ColorRGBA color,
                           std::vector<geometry_msgs::Point> positions);
  /**
   * @brief Destructor.
   */
  virtual ~ClickablePositionsMarker();

  /**
   * @brief Gets created marker with clickable positions.
   * @return marker with clickable positions
   */
  visualization_msgs::InteractiveMarker getMarker()
  {
    return interactive_marker_;
  }

  /**
   * @brief Click feedback
   */
  void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

private:
  void create();

private:
  // Interactive Marker server
  InteractiveMarkerServerPtr imServer_;

  std::string frame_id_;
  std::string topic_suffix_;
  float radius_;
  std_msgs::ColorRGBA color_;
  std::vector<geometry_msgs::Point> positions_;

  ros::NodeHandle nh_;
  visualization_msgs::InteractiveMarker interactive_marker_;
  std::stringstream marker_name_;
  std::map<std::string, geometry_msgs::Point> controls_;

  ros::Publisher click_publisher_;
};

} /* namespace srs_interaction_primitives */
#endif /* CLICKABLE_POSITIONS_H_ */
