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

#include "srs_interaction_primitives/clickable_positions/clickable_positions.h"

namespace srs_interaction_primitives
{

ClickablePositionsMarker::ClickablePositionsMarker(InteractiveMarkerServerPtr imServer, std::string frame_id,
                                                   std::string topic_suffix, float radius, std_msgs::ColorRGBA color,
                                                   std::vector<geometry_msgs::Point> positions)
{
  imServer_ = imServer;
  frame_id_ = frame_id;
  topic_suffix_ = topic_suffix;
  radius_ = radius;
  color_ = color;
  positions_ = positions;

  click_publisher_ = nh_.advertise<srs_interaction_primitives::PositionClicked>(BUT_PositionClicked_TOPIC(topic_suffix),
                                                                                1);

  create();
}

ClickablePositionsMarker::~ClickablePositionsMarker()
{
  click_publisher_.shutdown();
}

void ClickablePositionsMarker::create()
{

  interactive_marker_.header.frame_id = frame_id_;
  interactive_marker_.name = "click_positions_marker_" + topic_suffix_;

  for (unsigned int i = 0; i < positions_.size(); i++)
  {
    std::stringstream cname;
    cname << interactive_marker_.name << "_" << i;
    visualization_msgs::InteractiveMarkerControl im_control;
    im_control.name = cname.str();
    im_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    im_control.always_visible = true;
    visualization_msgs::Marker marker;
    marker.pose.position = positions_.at(i);
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.color = color_;
    marker.scale.x = radius_;
    marker.scale.y = radius_;
    marker.scale.z = radius_;
    im_control.markers.push_back(marker);
    interactive_marker_.controls.push_back(im_control);
    controls_.insert(std::make_pair(cname.str(), positions_.at(i)));
  }
}

void ClickablePositionsMarker::markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
  {
    ROS_INFO("Clicked");
    PositionClicked msg;
    msg.position = controls_[feedback->control_name];
    click_publisher_.publish(msg);
    imServer_->erase(interactive_marker_.name);
    imServer_->applyChanges();
  }
}

} /* namespace srs_interaction_primitives */
