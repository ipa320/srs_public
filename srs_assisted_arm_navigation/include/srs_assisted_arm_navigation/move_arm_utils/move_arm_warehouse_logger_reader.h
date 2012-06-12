/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  \author E. Gil Jones
 *********************************************************************/

#ifndef MOVE_ARM_WAREHOUSE_LOGGER_H_
#define MOVE_ARM_WAREHOUSE_LOGGER_H_

#include <mongo_ros/message_collection.h>

#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/MotionPlanRequest.h>
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <head_monitor_msgs/HeadMonitorFeedback.h>

namespace move_arm_warehouse
{

class MoveArmWarehouseLoggerReader {
  
public:

  MoveArmWarehouseLoggerReader();

  ~MoveArmWarehouseLoggerReader();

  const std::string& getHostname() const {
    return hostname_;
  }

  ///
  /// LOGGING FUNCTIONS
  ///

  void pushPlanningSceneToWarehouseWithoutId(const arm_navigation_msgs::PlanningScene& planning_scene,
                                    unsigned int& id); 

  void pushPlanningSceneToWarehouse(const arm_navigation_msgs::PlanningScene& planning_scene,
                                    const unsigned int ID);
  
  void pushMotionPlanRequestToWarehouse(const unsigned int planning_id, 
                                        const unsigned int mpr_id, 
                                        const std::string& stage_name,
                                        const arm_navigation_msgs::MotionPlanRequest& motion_plan_request);
    
  void pushJointTrajectoryToWarehouse(const unsigned int id,
                                      const std::string& trajectory_source,
                                      const ros::Duration& production_time, 
                                      const trajectory_msgs::JointTrajectory& trajectory,
                                      const trajectory_msgs::JointTrajectory& trajectory_control_error,
                                      const unsigned int ID,
                                      const unsigned int motion_plan_ID,
                                      const arm_navigation_msgs::ArmNavigationErrorCodes& error_code);
    
  void pushOutcomeToWarehouse(const unsigned int id,
                              const std::string& pipeline_stage,
                              const arm_navigation_msgs::ArmNavigationErrorCodes& error_codes);
  
  void pushPausedStateToWarehouse(const unsigned int id,
                                  const head_monitor_msgs::HeadMonitorFeedback& feedback);

  ///
  /// READING FUNCTIONS
  ///

  void getAvailablePlanningSceneList(const std::string& hostname, 
                                     std::vector<unsigned int>& planning_scene_ids_,
                                     std::vector<ros::Time>& creation_times);

  bool getPlanningScene(const std::string& hostname,
                        const unsigned int& id,
                        arm_navigation_msgs::PlanningScene& planning_scene,
                        std::string& hostname_out);

  bool getAssociatedOutcomes(const std::string& hostname,
                             const unsigned int planning_scene_id,
                             std::vector<std::string>& pipeline_names,
                             std::vector<arm_navigation_msgs::ArmNavigationErrorCodes>& error_codes);

  bool getAssociatedMotionPlanRequestsStageNames(const std::string& hostname, 
                                                 const unsigned int id, 
                                                 std::vector<std::string>& stage_names);

  bool getAssociatedMotionPlanRequest(const std::string& hostname, 
                                      const unsigned int planning_scene_id, 
                                      const unsigned int motion_plan_id, 
                                      arm_navigation_msgs::MotionPlanRequest& request);

  bool getAssociatedMotionPlanRequests(const std::string& hostname,
                                       const unsigned int planning_scene_id,
                                       std::vector<unsigned int>& IDs,
                                       std::vector<std::string>& stage_names,
                                       std::vector<arm_navigation_msgs::MotionPlanRequest>& requests);

  bool getAssociatedJointTrajectorySources(const std::string& hostname, 
                                           const unsigned int planning_scene_id,
                                           const unsigned int motion_request_id,
                                           std::vector<unsigned int>& ids,
                                           std::vector<std::string>& trajectory_sources);

  bool getAssociatedJointTrajectory(const std::string& hostname, 
                                    const unsigned int planning_scene_id,
                                    const unsigned int motion_plan_id,
                                    const unsigned int trajectory_id,
                                    ros::Duration& processing_time, 
                                    trajectory_msgs::JointTrajectory& joint_trajectory,
                                    trajectory_msgs::JointTrajectory& trajectory_control_error);

  bool getAssociatedJointTrajectories(const std::string& hostname,
                                      const unsigned int planning_scene_id,
                                      const unsigned int motion_plan_id,
                                      std::vector<trajectory_msgs::JointTrajectory>& trajectories,
                                      std::vector<trajectory_msgs::JointTrajectory>& trajectory_control_errors,
                                      std::vector<std::string>& sources,
                                      std::vector<unsigned int>& IDs,
                                      std::vector<ros::Duration>& durations,
                                      std::vector<int32_t>& error_codes);

  bool getAssociatedPausedStates(const std::string& hostname, 
                                 const unsigned int planning_scene_id,
                                 std::vector<ros::Time>& paused_times);

  bool getAssociatedPausedState(const std::string& hostname, 
                                const unsigned int planning_scene_id,
                                const ros::Time& paused_time,
                                head_monitor_msgs::HeadMonitorFeedback& paused_state);

  bool hasPlanningScene(const std::string& hostname,
                        const unsigned int id);

  bool removePlanningSceneAndAssociatedDataFromWarehouse(const std::string& hostname,
                                                         const unsigned int id);

  unsigned int determineNextPlanningSceneId();

protected:
  
  mongo_ros::Metadata initializeMetadataWithHostname();

  void addPlanningSceneTimeToMetadata(const arm_navigation_msgs::PlanningScene& planning_scene, mongo_ros::Metadata& metadata);
  void addPlanningSceneIdToMetadata(const unsigned int& id,
                                    mongo_ros::Metadata& metadata);


  mongo_ros::Query makeQueryForPlanningSceneTime(const ros::Time& time);
  mongo_ros::Query makeQueryForPlanningSceneId(const unsigned int id);

  // convenience functions for converting control error data
  std::string jointTrajectoryToString(const trajectory_msgs::JointTrajectory& trajectory);
  void stringToJointTrajectory(const std::string& trajectory, trajectory_msgs::JointTrajectory& joint_trajectory);

  mongo_ros::MessageCollection<arm_navigation_msgs::PlanningScene>* planning_scene_collection_;
  mongo_ros::MessageCollection<arm_navigation_msgs::MotionPlanRequest>* motion_plan_request_collection_;
  mongo_ros::MessageCollection<trajectory_msgs::JointTrajectory>* trajectory_collection_;
  mongo_ros::MessageCollection<arm_navigation_msgs::ArmNavigationErrorCodes>* outcome_collection_;
  mongo_ros::MessageCollection<head_monitor_msgs::HeadMonitorFeedback>* paused_state_collection_;
  
  std::string hostname_;
  
};

}
#endif
