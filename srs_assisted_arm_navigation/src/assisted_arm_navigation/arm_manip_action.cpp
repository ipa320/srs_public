/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Zdenek Materna (imaterna@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd/mm/2012
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

#include <srs_assisted_arm_navigation/assisted_arm_navigation/arm_manip_node.h>

using namespace srs_assisted_arm_navigation;
using namespace srs_assisted_arm_navigation_msgs;

void ManualArmManipActionServer::executeCB(const ManualArmManipGoalConstPtr &goal) {

  ros::Rate r(1);
  ros::Rate r5(5);
  ArmNavStart srv_start;

  result_.result.collision = false;
  result_.result.success = false;
  result_.result.repeat = false;
  result_.result.failed = false;
  result_.result.timeout = false;
  result_.result.time_elapsed = ros::Duration(0);

  ROS_INFO("Executing ManualArmManipAction");

  start_time_ = ros::Time::now();
  timeout_ = ros::Time::now();
  state_ = S_NONE;

  while(!ros::service::exists(SRV_START,true)) {

    ROS_INFO("Waiting for arm_nav_start service");
    r5.sleep();

  };

  // move arm to pre-grasp position or away
  /*if (((goal->pregrasp == true) ^ (goal->away == true)) == 1) {

    if (goal->pregrasp) {

    srv_start.request.pregrasp = true;
    srv_start.request.object_name = goal->object_name;

    } else {

      srv_start.request.away = true;

    }*/

  srv_start.request.allow_repeat = goal->allow_repeat;
  srv_start.request.action = goal->action;
  srv_start.request.object_name = goal->object_name;

    /// There should appear messagebox in RVIZ and user should start solving task.
    ros::service::call(SRV_START,srv_start);

    inited_ = true;

    ROS_INFO("Actionserver: starting looping");

    // 1 Hz loop
    while(ros::ok()) {

      state_ = new_state_;

      // test for sleeping user
      /*if ( (state_==S_NONE) && ((ros::Time::now()-start_time_) > start_timeout_) && (start_timeout_.toSec() > 0.0) ) {

        ROS_ERROR("%s: Canceled. Start timeout reached.", action_name_.c_str());

        result_.result.timeout = true;
        result_.result.time_elapsed = ros::Time::now() - start_time_;
        server_.setAborted(result_.result,"User is probably sleeping.");
        break;

      }*/

      // another test for sleeping user
      /*if (state_!=S_NONE && ((ros::Time::now()-timeout_) > solve_timeout_) && (solve_timeout_.toSec() > 0.0)) {

         ROS_ERROR("%s: Canceled. Solve timeout reached.", action_name_.c_str());

         result_.result.timeout = true;
         result_.result.time_elapsed = ros::Time::now() - start_time_;
         server_.setAborted(result_.result,"User is probably sleeping.");
         break;

      }*/

      // cancel action...
      if (server_.isPreemptRequested() || !ros::ok()) {

        ROS_INFO("%s: Preempted", action_name_.c_str());

        // TODO: clean things...

        result_.result.time_elapsed = ros::Time::now() - start_time_;
        server_.setPreempted(result_.result);

        break;

      }


      if ((state_ != S_SUCCESS) && (state_ != S_FAILED) && (state_ != S_REPEAT)) {

        // send feedback
        setFeedbackFalse();

        if (state_ == S_NEW) feedback_.feedback.starting = true;
        if (state_ == S_PLAN) feedback_.feedback.planning = true;
        if (state_ == S_EXECUTE) feedback_.feedback.executing = true;
        if (state_ == S_RESET) feedback_.feedback.reset = true;

        server_.publishFeedback(feedback_.feedback);

      } else {

        result_.result.failed = false;

        // send result
        result_.result.time_elapsed = ros::Time::now() - start_time_;

        if (state_ == S_SUCCESS) {

          result_.result.success = true;
          server_.setSucceeded(result_.result);

        }

        if (state_ == S_FAILED) {

          ROS_INFO("S_FAILED");
          result_.result.failed = true;
          server_.setAborted(result_.result);

        }

        if (state_ == S_REPEAT) {

          ROS_INFO("S_REPEAT");
          result_.result.repeat = true;
          server_.setAborted(result_.result);

        }


        break;

      }

      r.sleep();

    } // loop

    inited_ = false;
    new_state_ = S_NONE;

 /* // undefined
  } else {

    ROS_INFO("Can't move both to pregrasp pos. and away!");
     server_.setAborted(result_.result,"Can't move both to pregrasp pos. and away!");

  }*/

}


void ManualArmManipActionServer::srv_set_state(unsigned int n) {

  if (inited_) {
    new_state_ = n;
    ROS_DEBUG("Setting internal state to %u",n);
    timeout_ = ros::Time::now();
  }

}

void ManualArmManipActionServer::setFeedbackFalse() {

  feedback_.feedback.starting = false;
  feedback_.feedback.executing = false;
  feedback_.feedback.planning = false;
  feedback_.feedback.reset = false;

}
