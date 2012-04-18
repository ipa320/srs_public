/******************************************************************************
 * \file
 *
 * $Id: test_observers.cpp 625 2012-04-16 14:06:17Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 25/1/2012
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
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <std_msgs/String.h>
#include <srs_env_model/ScaleChanged.h>
#include <srs_env_model/PoseChanged.h>
#include <srs_env_model/MenuClicked.h>
#include <srs_env_model/MovementChanged.h>
#include <srs_env_model/TagChanged.h>
#include <srs_env_model/ContextChanged.h>

using namespace std;

/**
 * THIS IS ONLY A TESTING FILE!
 */

void scaleCallback(const srs_env_model::ScaleChangedConstPtr &marker_update)
{
  ROS_INFO("CALLBACK from marker %s", marker_update->marker_name.c_str());
  cout << marker_update->scale_change << endl;
  cout << marker_update->new_scale << endl;
}

void poseCallback(const srs_env_model::PoseChangedConstPtr &marker_update)
{
  ROS_INFO("CALLBACK from marker %s", marker_update->marker_name.c_str());
  cout << marker_update->pose_change << endl;
  cout << marker_update->new_pose << endl;
}

void menuCallback(const srs_env_model::MenuClickedConstPtr &marker_update)
{
  ROS_INFO("CALLBACK from marker %s", marker_update->marker_name.c_str());
  cout << marker_update->menu_title << endl;
  cout << marker_update->state << endl;
}

void movementCallback(const srs_env_model::MovementChangedConstPtr &marker_update)
{
  ROS_INFO("CALLBACK from marker %s", marker_update->marker_name.c_str());
  cout << marker_update->direction_change << endl;
  cout << marker_update->velocity_change << endl;
  cout << marker_update->new_direction << endl;
  cout << marker_update->new_velocity << endl;
}

void tagCallback(const srs_env_model::TagChangedConstPtr &marker_update)
{
  ROS_INFO("CALLBACK from marker %s", marker_update->marker_name.c_str());
  cout << marker_update->new_tag << endl;
}

void contextCallback(const srs_env_model::ContextChangedConstPtr &context_update)
{
  ROS_INFO("Context changed");
  cout << context_update->context << endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_observers");
  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe<srs_env_model::ScaleChanged> ("/but_gui/unknown_object/update/scale_changed", 20,
                                                                   scaleCallback);
  ros::Subscriber sub2 = n.subscribe<srs_env_model::PoseChanged> ("/but_gui/unknown_object/update/pose_changed", 20,
                                                                  poseCallback);
  ros::Subscriber sub3 = n.subscribe<srs_env_model::ScaleChanged> ("/but_gui/person_bbox/update/scale_changed", 20,
                                                                   scaleCallback);
  ros::Subscriber sub4 = n.subscribe<srs_env_model::PoseChanged> ("/but_gui/person_bbox/update/pose_changed", 20,
                                                                  poseCallback);
  ros::Subscriber sub5 = n.subscribe<srs_env_model::ScaleChanged> ("/but_gui/table_object/update/scale_changed", 20,
                                                                   scaleCallback);
  ros::Subscriber sub6 = n.subscribe<srs_env_model::PoseChanged> ("/but_gui/table_object/update/pose_changed", 20,
                                                                  poseCallback);

  ros::Subscriber subm1 = n.subscribe<srs_env_model::MenuClicked> ("/but_gui/table_object/update/menu_clicked", 20,
                                                                   menuCallback);
  ros::Subscriber subm2 = n.subscribe<srs_env_model::MenuClicked> ("/but_gui/person_bbox/update/menu_clicked", 20,
                                                                   menuCallback);
  ros::Subscriber subm3 = n.subscribe<srs_env_model::MenuClicked> ("/but_gui/unknown_object/update/menu_clicked", 20,
                                                                   menuCallback);
  ros::Subscriber subm4 = n.subscribe<srs_env_model::MenuClicked> ("/but_gui/milk_billboard/update/menu_clicked", 20,
                                                                   menuCallback);
  ros::Subscriber subm5 = n.subscribe<srs_env_model::MenuClicked> ("/but_gui/plane1/update/menu_clicked", 20,
                                                                   menuCallback);

  ros::Subscriber submovement =
      n.subscribe<srs_env_model::MovementChanged> ("/but_gui/milk_billboard/update/movement_changed", 20,
                                                   movementCallback);

  ros::Subscriber tag = n.subscribe<srs_env_model::TagChanged> ("/but_gui/plane/update/tag_changed", 20, tagCallback);

  ros::Subscriber milk1 = n.subscribe<srs_env_model::MenuClicked> ("/but_gui/milk/update/menu_clicked", 20,
                                                                   menuCallback);
  ros::Subscriber milk2 = n.subscribe<srs_env_model::PoseChanged> ("/but_gui/milk/update/pose_changed", 20,
                                                                   poseCallback);
  ros::Subscriber milk3 = n.subscribe<srs_env_model::ScaleChanged> ("/but_gui/milk/update/scale_changed", 20,
                                                                    scaleCallback);

  // Test context changes
  ros::Subscriber context = n.subscribe<srs_env_model::ContextChanged> ("but_context/context_changed", 20,
                                                                        contextCallback);

  ros::spin();
}
