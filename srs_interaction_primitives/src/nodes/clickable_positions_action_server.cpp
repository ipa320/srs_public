/******************************************************************************
 * \file
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
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

#include <ros/ros.h>
#include <srs_interaction_primitives/ClickablePositionsAction.h>
#include <srs_interaction_primitives/ClickablePositions.h>
#include <srs_interaction_primitives/topics_list.h>
#include <srs_interaction_primitives/services_list.h>
#include <actionlib/server/simple_action_server.h>
#include <map>

class ClickablePositionsAction
{
public:

  ClickablePositionsAction(std::string name) :
      as_(nh_, name, false), action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&ClickablePositionsAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ClickablePositionsAction::preemptCB, this));
    as_.start();

    clickable_positions_client_ = nh_.serviceClient<srs_interaction_primitives::ClickablePositions>(
        srs_interaction_primitives::ClickablePositions_SRV);

    ROS_INFO("Clickable Positions Action Server ready");
  }

  ~ClickablePositionsAction(void)
  {
  }

  void goalCB()
  {
    // accept the new goal
    ROS_INFO("New goal");
    goal_ = as_.acceptNewGoal();

    std::string topic_suffix = BUT_PositionClicked_TOPIC(goal_->topic_suffix);
    position_clicked_subscriber_ = nh_.subscribe(topic_suffix, 1, &ClickablePositionsAction::positionClickedCallback, this);

    srs_interaction_primitives::ClickablePositions srv;
    srv.request.color = goal_->color;
    srv.request.frame_id = goal_->frame_id;
    srv.request.positions = goal_->positions;
    srv.request.radius = goal_->radius;
    srv.request.topic_suffix = goal_->topic_suffix;

    ros::service::waitForService(srs_interaction_primitives::ClickablePositions_SRV);

    if (clickable_positions_client_.call(srv))
    {

    }
    else
    {
      ROS_ERROR("Clickable positions not added");
      as_.setAborted();
    }

  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void positionClickedCallback(const srs_interaction_primitives::PositionClickedPtr &update)
  {
    result_.clicked_position = update->position;
    as_.publishFeedback(feedback_);
  }

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient clickable_positions_client_;
  ros::Subscriber position_clicked_subscriber_;
  std::string action_name_;
  actionlib::SimpleActionServer<srs_interaction_primitives::ClickablePositionsAction> as_;
  srs_interaction_primitives::ClickablePositionsGoalConstPtr goal_;
  srs_interaction_primitives::ClickablePositionsFeedback feedback_;
  srs_interaction_primitives::ClickablePositionsResult result_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clickable_positions_action_server");

  ClickablePositionsAction clickable_positions("clickable_positions_server");
  ros::spin();

  return 0;
}
