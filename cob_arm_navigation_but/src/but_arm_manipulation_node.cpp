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

#include "but_arm_manipulation_node.h"



using namespace arm_navigation_msgs;
using namespace planning_scene_utils;

/**
 *  \todo use controllerDoneCallback / armHasStoppedMoving to check if the arm finished trajectory
 *
 */


CArmManipulationEditor::CArmManipulationEditor(planning_scene_utils::PlanningSceneParameters& params, std::vector<string> clist) : PlanningSceneEditor(params)
{

    action_server_ptr_ = NULL;
    inited = false;
    params_ = params;

    links_ = clist;

}

CArmManipulationEditor::~CArmManipulationEditor() {

  if (action_server_ptr_!=NULL) delete action_server_ptr_;

  if (tfl_!=NULL) delete tfl_;

}

void CArmManipulationEditor::onPlanningSceneLoaded() {};
void CArmManipulationEditor::updateState() {};
void CArmManipulationEditor::planCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode) {

  if(errorCode.val != ArmNavigationErrorCodes::SUCCESS)
    {

    ROS_ERROR("Planning failed with error: %s (%d)",armNavigationErrorCodeToString(errorCode).c_str(), errorCode.val);


    }

};
void CArmManipulationEditor::filterCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode) {

  if(errorCode.val != ArmNavigationErrorCodes::SUCCESS)
    {

    ROS_ERROR("Filtering failed with error: %s (%d)",armNavigationErrorCodeToString(errorCode).c_str(),errorCode.val);


    }

};
void CArmManipulationEditor::attachObjectCallback(const std::string& name) {};
void CArmManipulationEditor::selectedTrajectoryCurrentPointChanged( unsigned int new_current_point ) {};




void CArmManipulationEditor::armHasStoppedMoving() {

  ROS_INFO("Arm has stopped moving :)");

}


void CArmManipulationEditor::reset() {



  std::vector<std::string>::iterator tmp;

  for(tmp = coll_obj_attached_id.begin(); tmp != coll_obj_attached_id.end(); tmp++) {

    if (*tmp != "") deleteCollisionObject(*tmp);

    }

  coll_obj_attached_id.clear();

  std::vector<t_det_obj>::iterator tmpo;

  for(tmpo = coll_obj_det.begin(); tmpo != coll_obj_det.end(); tmpo++) {

    if ((*tmpo).id != "") deleteCollisionObject((*tmpo).id);

    }

  //coll_obj_det.clear();


  // this should delete planning scene from warehouse and also all associated data...
  if (planning_scene_id!="") {

    /*std::vector<unsigned int> erased_trajectories;
    deleteMotionPlanRequest(mpr_id,erased_trajectories);*/

    PlanningSceneData& data = planning_scene_map_[planning_scene_id];
    if (move_arm_warehouse_logger_reader_->removePlanningSceneAndAssociatedDataFromWarehouse(data.getHostName(), data.getId())) {

      ROS_INFO("Planning scene was removed from warehouse.");

      planning_scene_map_.erase(planning_scene_id);
      planning_scene_id = "";
      mpr_id = 0;
      inited = false;

    } else {

      ROS_ERROR("Error on removing planning scene from warehouse.");

    }

  } else ROS_WARN("Cannot remove planning scene: it has been already removed");

}



void CArmManipulationEditor::spin_callback(const ros::TimerEvent&)
{

  if (inited==true) sendMarkers();


}


int main(int argc, char** argv)
{

      CArmManipulationEditor * ps_editor = NULL;
      planning_scene_utils::PlanningSceneParameters params;


      ROS_INFO("Starting");
      ros::init(argc, argv, "but_simple_manual_arm_navigation");

      ros::param::param<std::string>("set_planning_scene_diff_name", params.set_planning_scene_diff_name_, SET_PLANNING_SCENE_DIFF_NAME);
      ros::param::param<std::string>("left_ik_name", params.left_ik_name_, LEFT_IK_NAME);
      ros::param::param<std::string>("left_interpolate_service_name", params.left_interpolate_service_name_, LEFT_INTERPOLATE_SERVICE_NAME);
      ros::param::param<std::string>("non_coll_left_ik_name", params.non_coll_left_ik_name_, NON_COLL_LEFT_IK_NAME);
      ros::param::param<std::string>("non_coll_right_ik_name", params.non_coll_right_ik_name_, NON_COLL_RIGHT_IK_NAME);
      ros::param::param<std::string>("planner_1_service_name", params.planner_1_service_name_, PLANNER_1_SERVICE_NAME);
      ros::param::param<std::string>("planner_2_service_name", params.planner_2_service_name_, PLANNER_2_SERVICE_NAME);
      ros::param::param<std::string>("proximity_space_planner_name", params.proximity_space_planner_name_, PROXIMITY_SPACE_PLANNER_NAME);
      ros::param::param<std::string>("proximity_space_service_name",  params.proximity_space_service_name_, PROXIMITY_SPACE_SERVICE_NAME);
      ros::param::param<std::string>("proximity_space_validity_name",  params.proximity_space_validity_name_,  PROXIMITY_SPACE_VALIDITY_NAME);
      ros::param::param<std::string>("right_ik_name", params.right_ik_name_, RIGHT_IK_NAME);
      ros::param::param<std::string>("right_interpolate_service_name", params.right_interpolate_service_name_, RIGHT_INTERPOLATE_SERVICE_NAME);
      ros::param::param<std::string>("trajectory_filter_1_service_name", params.trajectory_filter_1_service_name_, TRAJECTORY_FILTER_1_SERVICE_NAME);
      ros::param::param<std::string>("trajectory_filter_2_service_name", params.trajectory_filter_2_service_name_, TRAJECTORY_FILTER_2_SERVICE_NAME);
      ros::param::param<std::string>("vis_topic_name", params.vis_topic_name_ , VIS_TOPIC_NAME);
      ros::param::param<std::string>("right_ik_link", params.right_ik_link_ , RIGHT_IK_LINK);
      ros::param::param<std::string>("left_ik_link", params.left_ik_link_ , LEFT_IK_LINK);
      ros::param::param<std::string>("right_arm_group", params.right_arm_group_ , RIGHT_ARM_GROUP);
      ros::param::param<std::string>("left_arm_group", params.left_arm_group_ , LEFT_ARM_GROUP);
      ros::param::param<std::string>("right_redundancy", params.right_redundancy_ , RIGHT_ARM_REDUNDANCY);
      ros::param::param<std::string>("left_redundancy", params.left_redundancy_ , LEFT_ARM_REDUNDANCY);
      ros::param::param<std::string>("execute_left_trajectory", params.execute_left_trajectory_ , EXECUTE_LEFT_TRAJECTORY);
      ros::param::param<std::string>("execute_right_trajectory", params.execute_right_trajectory_ , EXECUTE_RIGHT_TRAJECTORY);
      ros::param::param<std::string>("list_controllers_service", params.list_controllers_service_, LIST_CONTROLLERS_SERVICE);
      ros::param::param<std::string>("load_controllers_service", params.load_controllers_service_, LOAD_CONTROLLERS_SERVICE);
      ros::param::param<std::string>("unload_controllers_service", params.unload_controllers_service_, UNLOAD_CONTROLLERS_SERVICE);
      ros::param::param<std::string>("switch_controllers_service", params.switch_controllers_service_, SWITCH_CONTROLLERS_SERVICE);
      ros::param::param<std::string>("gazebo_robot_model", params.gazebo_model_name_, GAZEBO_ROBOT_MODEL);
      ros::param::param<std::string>("robot_description_param", params.robot_description_param_, ROBOT_DESCRIPTION_PARAM);
      ros::param::param<bool>("use_robot_data", params.use_robot_data_, true);
      params.sync_robot_state_with_gazebo_ = false;


      std::vector<std::string> links;

      // TODO move to configuration file
      links.push_back("arm_7_link");
      links.push_back("sdh_palm_link");
      links.push_back("sdh_grasp_link");
      links.push_back("sdh_tip_link");

      links.push_back("sdh_finger_21_link");
      links.push_back("sdh_finger_22_link");
      links.push_back("sdh_finger_23_link");

      links.push_back("sdh_finger_11_link");
      links.push_back("sdh_finger_12_link");
      links.push_back("sdh_finger_13_link");

      links.push_back("sdh_thumb_1_link");
      links.push_back("sdh_thumb_2_link");
      links.push_back("sdh_thumb_3_link");

      ROS_INFO("Creating planning scene editor");
      ps_editor = new CArmManipulationEditor(params,links);

      ROS_INFO("Advertising services");
      ros::NodeHandle n;
      ros::ServiceServer service_new = n.advertiseService(SRV_NEW, &CArmManipulationEditor::ArmNavNew,ps_editor);
      ros::ServiceServer service_plan = n.advertiseService(SRV_PLAN, &CArmManipulationEditor::ArmNavPlan,ps_editor);
      ros::ServiceServer service_play = n.advertiseService(SRV_PLAY, &CArmManipulationEditor::ArmNavPlay,ps_editor);
      ros::ServiceServer service_execute = n.advertiseService(SRV_EXECUTE, &CArmManipulationEditor::ArmNavExecute,ps_editor);
      ros::ServiceServer service_reset = n.advertiseService(SRV_RESET, &CArmManipulationEditor::ArmNavReset,ps_editor);
      ros::ServiceServer service_refresh = n.advertiseService(SRV_REFRESH, &CArmManipulationEditor::ArmNavRefresh,ps_editor);

      ros::ServiceServer service_success = n.advertiseService(SRV_SUCCESS, &CArmManipulationEditor::ArmNavSuccess,ps_editor);
      ros::ServiceServer service_failed = n.advertiseService(SRV_FAILED, &CArmManipulationEditor::ArmNavFailed,ps_editor);

      ros::ServiceServer service_collobj = n.advertiseService(SRV_COLLOBJ, &CArmManipulationEditor::ArmNavCollObj,ps_editor);

      ros::Timer timer1 = n.createTimer(ros::Duration(0.01), &CArmManipulationEditor::spin_callback,ps_editor);

      ManualArmManipActionServer act_server(ACT_ARM_MANIP);

      double tmp;

      ros::param::param<double>("start_timeout",tmp,START_TIMEOUT);
      if (tmp>0) act_server.start_timeout_ = ros::Duration(tmp);
      else act_server.start_timeout_ = ros::Duration(START_TIMEOUT);

      ros::param::param<double>("solve_timeout",tmp,SOLVE_TIMEOUT);
      if (tmp>0) act_server.solve_timeout_ = ros::Duration(tmp);
      else act_server.solve_timeout_ = ros::Duration(SOLVE_TIMEOUT);

      ps_editor->action_server_ptr_ = &act_server;

      ps_editor->tfl_ = new tf::TransformListener();


      ROS_INFO("Spinning");

      ros::spin();


      ros::shutdown();

      delete ps_editor;

}
