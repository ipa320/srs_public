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

#include <ros/console.h>
#include <boost/foreach.hpp>
#include <sstream>

//#include <move_arm_warehouse/move_arm_warehouse_logger_reader.h>
#include "srs_assisted_arm_navigation/move_arm_utils/move_arm_warehouse_logger_reader.h"

using namespace move_arm_warehouse;

static const std::string DATABASE_NAME="arm_navigation";
static const std::string PLANNING_SCENE_TIME_NAME="planning_scene_time";
static const std::string PLANNING_SCENE_ID_NAME="planning_scene_id";
static const std::string MOTION_PLAN_REQUEST_ID_NAME="motion_request_id";
static const std::string TRAJECTORY_ID_NAME="trajectory_id";
static const std::string TRAJECTORY_MOTION_REQUEST_ID_NAME="trajectory_motion_request_id";
static const std::string PAUSED_COLLISION_MAP_TIME_NAME="paused_collision_map_time";

typedef mongo_ros::MessageWithMetadata<arm_navigation_msgs::PlanningScene>::ConstPtr PlanningSceneWithMetadata;
typedef mongo_ros::MessageWithMetadata<arm_navigation_msgs::MotionPlanRequest>::ConstPtr MotionPlanRequestWithMetadata;
typedef mongo_ros::MessageWithMetadata<trajectory_msgs::JointTrajectory>::ConstPtr JointTrajectoryWithMetadata;
typedef mongo_ros::MessageWithMetadata<arm_navigation_msgs::ArmNavigationErrorCodes>::ConstPtr ErrorCodesWithMetadata;
typedef mongo_ros::MessageWithMetadata<head_monitor_msgs::HeadMonitorFeedback>::ConstPtr HeadMonitorFeedbackWithMetadata;
  
MoveArmWarehouseLoggerReader::MoveArmWarehouseLoggerReader()
{
  char hostname[256];
  
  gethostname(hostname, 256);

  hostname_ = hostname;

  ROS_DEBUG_STREAM("Hostname is " << hostname_);


  planning_scene_collection_ = new mongo_ros::MessageCollection<arm_navigation_msgs::PlanningScene>(DATABASE_NAME, "planning_scene");
  motion_plan_request_collection_ = new mongo_ros::MessageCollection<arm_navigation_msgs::MotionPlanRequest>(DATABASE_NAME, "motion_plan_request");
  trajectory_collection_ = new mongo_ros::MessageCollection<trajectory_msgs::JointTrajectory>(DATABASE_NAME, "trajectory");
  outcome_collection_ = new mongo_ros::MessageCollection<arm_navigation_msgs::ArmNavigationErrorCodes>(DATABASE_NAME, "outcome");
  paused_state_collection_ = new mongo_ros::MessageCollection<head_monitor_msgs::HeadMonitorFeedback>(DATABASE_NAME, "paused_state");
}

MoveArmWarehouseLoggerReader::~MoveArmWarehouseLoggerReader() {
  delete planning_scene_collection_;
  delete motion_plan_request_collection_;
  delete trajectory_collection_;
  delete outcome_collection_;
  delete paused_state_collection_;
}

///
/// LOGGING FUNCTIONS
///

mongo_ros::Metadata MoveArmWarehouseLoggerReader::initializeMetadataWithHostname()
{
  return mongo_ros::Metadata("hostname", hostname_);
}

void MoveArmWarehouseLoggerReader::addPlanningSceneIdToMetadata(const unsigned int& id,
                                                                mongo_ros::Metadata& metadata) {
  metadata.append(PLANNING_SCENE_ID_NAME, id);
}

void MoveArmWarehouseLoggerReader::addPlanningSceneTimeToMetadata(const arm_navigation_msgs::PlanningScene& planning_scene,
                                                                  mongo_ros::Metadata& metadata)
{
  metadata.append(PLANNING_SCENE_TIME_NAME, planning_scene.robot_state.joint_state.header.stamp.toSec());
}

unsigned int MoveArmWarehouseLoggerReader::determineNextPlanningSceneId() {
  mongo_ros::Query q;
  std::vector<PlanningSceneWithMetadata> planning_scenes = planning_scene_collection_->pullAllResults(q, true, PLANNING_SCENE_ID_NAME, false);
  if(planning_scenes.size() == 0) {
    return 0;
  }
  //should be in reverse order
  return planning_scenes.front()->lookupInt(PLANNING_SCENE_ID_NAME)+1;
}

void MoveArmWarehouseLoggerReader::pushPlanningSceneToWarehouseWithoutId(const arm_navigation_msgs::PlanningScene& planning_scene,
                                                                         unsigned int& id) 
{
  id = determineNextPlanningSceneId();
  pushPlanningSceneToWarehouse(planning_scene, id);
}


void MoveArmWarehouseLoggerReader::pushPlanningSceneToWarehouse(const arm_navigation_msgs::PlanningScene& planning_scene,
                                                                const unsigned int id)
{
  mongo_ros::Metadata metadata = initializeMetadataWithHostname();
  addPlanningSceneTimeToMetadata(planning_scene, metadata);
  //TODO - check for duplicates?
  addPlanningSceneIdToMetadata(id, metadata);
  planning_scene_collection_->insert(planning_scene, metadata);
}

void MoveArmWarehouseLoggerReader::pushMotionPlanRequestToWarehouse(const unsigned int planning_scene_id,
                                                                    const unsigned int mpr_id,
                                                                    const std::string& stage_name,
                                                                    const arm_navigation_msgs::MotionPlanRequest& motion_plan_request)
{
  mongo_ros::Metadata metadata = initializeMetadataWithHostname();
  addPlanningSceneIdToMetadata(planning_scene_id, metadata);
  
  metadata.append("stage_name", stage_name);
  
  metadata.append(MOTION_PLAN_REQUEST_ID_NAME, mpr_id);
  //adding the presence of goal pose constraints to metadata
  metadata.append("has_goal_position_constraints", !motion_plan_request.goal_constraints.position_constraints.empty());

 
  metadata.append("has_path_constraints", 
                  (!motion_plan_request.path_constraints.orientation_constraints.empty() || motion_plan_request.path_constraints.position_constraints.empty()));
  
  motion_plan_request_collection_->insert(motion_plan_request, metadata);
}

void MoveArmWarehouseLoggerReader::pushJointTrajectoryToWarehouse(const unsigned int planning_scene_id,
                                                                  const std::string& trajectory_source,
                                                                  const ros::Duration& production_time,
                                                                  const trajectory_msgs::JointTrajectory& trajectory,
                                                                  const trajectory_msgs::JointTrajectory& trajectory_control_error,
                                                                  const unsigned int trajectory_id,
                                                                  const unsigned int motion_request_id,
                                                                  const arm_navigation_msgs::ArmNavigationErrorCodes& error_code)
{
  mongo_ros::Metadata metadata = initializeMetadataWithHostname();
  addPlanningSceneIdToMetadata(planning_scene_id, metadata);

  metadata.append("trajectory_source", trajectory_source);
  metadata.append("production_time", production_time.toSec());
  metadata.append(TRAJECTORY_ID_NAME, trajectory_id);
  metadata.append(TRAJECTORY_MOTION_REQUEST_ID_NAME, motion_request_id);
  metadata.append("trajectory_error_code", error_code.val);
  metadata.append("controller_error", jointTrajectoryToString(trajectory_control_error));
  trajectory_collection_->insert(trajectory, metadata);
}

void MoveArmWarehouseLoggerReader::pushOutcomeToWarehouse(const unsigned int id, 
                                                          const std::string& pipeline_stage,
                                                          const arm_navigation_msgs::ArmNavigationErrorCodes& error_codes)
{
  mongo_ros::Metadata metadata = initializeMetadataWithHostname();
  addPlanningSceneIdToMetadata(id, metadata);

  metadata.append("pipeline_stage", pipeline_stage);
  outcome_collection_->insert(error_codes, metadata);
}

void MoveArmWarehouseLoggerReader::pushPausedStateToWarehouse(const unsigned int id, 
                                                              const head_monitor_msgs::HeadMonitorFeedback& feedback)
{
  mongo_ros::Metadata metadata = initializeMetadataWithHostname();
  addPlanningSceneIdToMetadata(id, metadata);
  metadata.append(PAUSED_COLLISION_MAP_TIME_NAME, feedback.paused_collision_map.header.stamp.toSec());
  paused_state_collection_->insert(feedback, metadata);
}

///
/// READING FUNCTIONS
///

void MoveArmWarehouseLoggerReader::getAvailablePlanningSceneList(const std::string& hostname, 
                                                                 std::vector<unsigned int>& planning_scene_ids,
                                                                 std::vector<ros::Time>& creation_times)
{
  creation_times.clear();
  planning_scene_ids.clear();

  // std::stringstream fin(planning_scenes[i]->metadata);
  // YAML::Parser parser(fin);
  // YAML::Node doc;
  // while(parser.GetNextDocument(doc)) {    }

  mongo_ros::Query q;
  std::vector<PlanningSceneWithMetadata> planning_scenes = planning_scene_collection_->pullAllResults(q, true, PLANNING_SCENE_TIME_NAME, true);

  planning_scene_ids.resize(planning_scenes.size());
  creation_times.resize(planning_scenes.size());

  bool no_ids = false;
  for(unsigned int i = 0; i < planning_scenes.size(); i++) {    
    if(!planning_scenes[i]->metadata.hasField(PLANNING_SCENE_ID_NAME.c_str())) {
      no_ids = true;
      break;
    }
    planning_scene_ids[i] = planning_scenes[i]->lookupInt(PLANNING_SCENE_ID_NAME);
    ROS_DEBUG_STREAM("Got planning scene id " << planning_scene_ids[i]);
    creation_times[i] = ros::Time(planning_scenes[i]->lookupDouble(PLANNING_SCENE_TIME_NAME));
  }
  if(no_ids) {
    ROS_WARN_STREAM("No planning scene ids, can't load");
  }
}

mongo_ros::Query MoveArmWarehouseLoggerReader::makeQueryForPlanningSceneTime(const ros::Time& time)
{
  mongo_ros::Query retq;
  retq.append(PLANNING_SCENE_TIME_NAME, time.toSec()); 
  return retq;
}

mongo_ros::Query MoveArmWarehouseLoggerReader::makeQueryForPlanningSceneId(const unsigned int id)
{
  mongo_ros::Query retq;
  retq.append(PLANNING_SCENE_ID_NAME, id); 
  return retq;
}

bool MoveArmWarehouseLoggerReader::getPlanningScene(const std::string& hostname, 
                                                    const unsigned int& id,
                                                    arm_navigation_msgs::PlanningScene& planning_scene, 
                                                    std::string& hostname_out)
{
  mongo_ros::Query q = makeQueryForPlanningSceneId(id);  
  std::vector<PlanningSceneWithMetadata> planning_scenes = planning_scene_collection_->pullAllResults(q, false);

  if(planning_scenes.size() == 0) {
    ROS_WARN_STREAM("No scenes with id " << id);
    return false;
  } else if(planning_scenes.size() > 1) {
    ROS_WARN_STREAM("More than one scene with id " << id << " num " << planning_scenes.size());
  }

  PlanningSceneWithMetadata& scene = planning_scenes[0];
  planning_scene = *scene;
  hostname_out = scene->lookupString("hostname");
  return true;
}

bool MoveArmWarehouseLoggerReader::getAssociatedOutcomes(const std::string& hostname,
                                                         const unsigned int id, 
                                                         std::vector<std::string>& pipeline_names,
                                                         std::vector<arm_navigation_msgs::ArmNavigationErrorCodes>& error_codes)
{
  mongo_ros::Query q = makeQueryForPlanningSceneId(id);  
  std::vector<ErrorCodesWithMetadata> meta_error_codes = outcome_collection_->pullAllResults(q, false);
  
  if(meta_error_codes.size() == 0) {
    ROS_DEBUG_STREAM("No outcomes associated with id " << id);
    return false;
  } 
  error_codes.resize(meta_error_codes.size());
  pipeline_names.resize(meta_error_codes.size());
  for(unsigned int i = 0; i < meta_error_codes.size(); i++) {
    pipeline_names[i] = meta_error_codes[i]->lookupString("pipeline_stage");
    error_codes[i] = *meta_error_codes[i];
  }
  return true;
}
                                                 
bool MoveArmWarehouseLoggerReader::getAssociatedMotionPlanRequestsStageNames(const std::string& hostname, 
                                                                             const unsigned int planning_scene_id,
                                                                             std::vector<std::string>& stage_names)
{
  mongo_ros::Query q = makeQueryForPlanningSceneId(planning_scene_id);  
  std::vector<MotionPlanRequestWithMetadata> motion_plan_requests = motion_plan_request_collection_->pullAllResults(q, true, PLANNING_SCENE_ID_NAME, true);

  if(motion_plan_requests.size() == 0) {
    ROS_DEBUG_STREAM("No motion plan requests with id" << planning_scene_id);
    return false;
  } 

  stage_names.resize(motion_plan_requests.size());
  for(unsigned int i = 0; i < motion_plan_requests.size(); i++) {
    stage_names[i] = motion_plan_requests[i]->lookupString("stage_name");
  }
  return true; 
}

bool MoveArmWarehouseLoggerReader::getAssociatedMotionPlanRequest(const std::string& hostname, 
                                                                  const unsigned int planning_scene_id, 
                                                                  const unsigned int motion_request_id,
                                                                  arm_navigation_msgs::MotionPlanRequest& request)
{  
  mongo_ros::Query q = makeQueryForPlanningSceneId(planning_scene_id);  
  q.append(MOTION_PLAN_REQUEST_ID_NAME, motion_request_id);
  std::vector<MotionPlanRequestWithMetadata> motion_plan_requests = motion_plan_request_collection_->pullAllResults(q, false);

  if(motion_plan_requests.size() == 0) {
    ROS_WARN_STREAM("No motion plan requests with planning scene id " << planning_scene_id
                    << " and motion plan id " << motion_request_id);
    return false;
  } else if(motion_plan_requests.size() > 1) {
    ROS_WARN_STREAM("More than one motion plan requests with planning scene id " << planning_scene_id
                    << " and motion plan id " << motion_request_id);
    return false;
  }
  request = *motion_plan_requests[0];
  return true;
}

bool MoveArmWarehouseLoggerReader::getAssociatedMotionPlanRequests(const std::string& hostname,
                                                                   const unsigned int planning_scene_id, 
                                                                   std::vector<unsigned int>& ids,
                                                                   std::vector<std::string>& stage_names,
                                                                   std::vector<arm_navigation_msgs::MotionPlanRequest>& requests)
{
  stage_names.clear();
  ids.clear();
  requests.clear();
  mongo_ros::Query q = makeQueryForPlanningSceneId(planning_scene_id);
  std::vector<MotionPlanRequestWithMetadata> motion_plan_requests = motion_plan_request_collection_->pullAllResults(q, false);

  for(size_t i = 0; i < motion_plan_requests.size(); i++)
  {
    stage_names.push_back(motion_plan_requests[i]->lookupString("stage_name"));
    ids.push_back(motion_plan_requests[i]->lookupInt(MOTION_PLAN_REQUEST_ID_NAME));
    requests.push_back(*motion_plan_requests[i]);
    ROS_DEBUG_STREAM("Loading planning scene " << planning_scene_id << " motion plan request " << ids[i] << " from warehouse...");
  }

  return true;
}

bool MoveArmWarehouseLoggerReader::getAssociatedJointTrajectorySources(const std::string& hostname, 
                                                                       const unsigned int planning_scene_id,
                                                                       const unsigned int motion_request_id,
                                                                       std::vector<unsigned int>& ids, 
                                                                       std::vector<std::string>& trajectory_sources)
{
  ids.clear();
  trajectory_sources.clear();
  mongo_ros::Query q = makeQueryForPlanningSceneId(planning_scene_id);
  q.append(TRAJECTORY_MOTION_REQUEST_ID_NAME, motion_request_id);
  
  std::vector<JointTrajectoryWithMetadata> joint_trajectories = trajectory_collection_->pullAllResults(q, true, TRAJECTORY_ID_NAME);

  if(joint_trajectories.size() == 0) {
    ROS_WARN_STREAM("No joint trajectories with planning scene id " << planning_scene_id
                    << " and motion plan id " << motion_request_id);
    return false;
  } 
  ids.resize(joint_trajectories.size());
  trajectory_sources.resize(joint_trajectories.size());
  for(unsigned int i = 0; i < joint_trajectories.size(); i++) {
    trajectory_sources[i] = joint_trajectories[i]->lookupString("trajectory_source");
  }
  return true; 
}

bool MoveArmWarehouseLoggerReader::getAssociatedJointTrajectory(const std::string& hostname, 
                                                                const unsigned int planning_scene_id,
                                                                const unsigned int motion_request_id,
                                                                const unsigned int trajectory_id,
                                                                ros::Duration& duration, 
                                                                trajectory_msgs::JointTrajectory& joint_trajectory,
                                                                trajectory_msgs::JointTrajectory& trajectory_control_error)
{
  mongo_ros::Query q = makeQueryForPlanningSceneId(planning_scene_id);  
  q.append(TRAJECTORY_MOTION_REQUEST_ID_NAME, motion_request_id);
  q.append(TRAJECTORY_ID_NAME, trajectory_id);
  std::vector<JointTrajectoryWithMetadata> joint_trajectories = trajectory_collection_->pullAllResults(q, false);

  if(joint_trajectories.size() == 0) {
    ROS_WARN_STREAM("No joint trajectories with with planning scene id " << planning_scene_id
                    << " and motion plan id " << motion_request_id
                    << " and trajectory id " << trajectory_id);
    return false;
  } else if(joint_trajectories.size() > 1) {
    ROS_WARN_STREAM("Multiple trajectories in db with same ids");
    return false;
  }
  
  duration = ros::Duration(joint_trajectories[0]->lookupDouble("production_time"));
  joint_trajectory = *joint_trajectories[0];
  stringToJointTrajectory(joint_trajectories[0]->lookupString("controller_error"),trajectory_control_error);
  return true;
}

bool MoveArmWarehouseLoggerReader::getAssociatedJointTrajectories(const std::string& hostname,
                                                                  const unsigned int planning_scene_id,
                                                                  const unsigned int motion_request_id,
                                                                  std::vector<trajectory_msgs::JointTrajectory>& trajectories,
                                                                  std::vector<trajectory_msgs::JointTrajectory>& trajectory_control_errors,
                                                                  std::vector<std::string>& sources,
                                                                  std::vector<unsigned int>& ids,
                                                                  std::vector<ros::Duration>& durations,
                                                                  std::vector<int32_t>& error_codes)
{
  trajectories.clear();
  trajectory_control_errors.clear();
  sources.clear();
  ids.clear();
  durations.clear();
  error_codes.clear();
  mongo_ros::Query q = makeQueryForPlanningSceneId(planning_scene_id);
  q.append(TRAJECTORY_MOTION_REQUEST_ID_NAME, motion_request_id);
  std::vector<JointTrajectoryWithMetadata> joint_trajectories = trajectory_collection_->pullAllResults(q, false);
 trajectory_msgs::JointTrajectory trajectory_control_error;

  for(size_t i = 0; i < joint_trajectories.size(); i++)
  {
    trajectories.push_back(*joint_trajectories[i]);
    stringToJointTrajectory(joint_trajectories[i]->lookupString("controller_error"),trajectory_control_error);
    trajectory_control_errors.push_back(trajectory_control_error);
    sources.push_back(joint_trajectories[i]->lookupString("trajectory_source"));
    ids.push_back(joint_trajectories[i]->lookupInt(TRAJECTORY_ID_NAME));
    ROS_DEBUG_STREAM("Loading mpr id " << motion_request_id << " trajectory " << ids[i] << " from warehouse...");
    durations.push_back(ros::Duration(joint_trajectories[i]->lookupDouble("production_time")));
    error_codes.push_back(joint_trajectories[i]->lookupInt("trajectory_error_code"));
  }

  return true;
}

bool MoveArmWarehouseLoggerReader::getAssociatedPausedStates(const std::string& hostname, 
                                                             const unsigned int id, 
                                                             std::vector<ros::Time>& paused_times)
{
  paused_times.clear();
  mongo_ros::Query q = makeQueryForPlanningSceneId(id);  
  std::vector<HeadMonitorFeedbackWithMetadata> paused_states = paused_state_collection_->pullAllResults(q, true, PLANNING_SCENE_ID_NAME);

  if(paused_states.size() == 0) {
    return false;
  } 
  paused_times.resize(paused_states.size());
  for(unsigned int i = 0; i < paused_states.size(); i++) {
    paused_times[i] = ros::Time(paused_states[i]->lookupDouble("paused_collision_map_stamp"));
  }
  return true;   
}

bool MoveArmWarehouseLoggerReader::getAssociatedPausedState(const std::string& hostname, 
                                                            const unsigned int id, 
                                                            const ros::Time& paused_time,
                                                            head_monitor_msgs::HeadMonitorFeedback& paused_state)
{
  mongo_ros::Query q = makeQueryForPlanningSceneId(id);  
  q.append(PAUSED_COLLISION_MAP_TIME_NAME, paused_time.toSec());

  std::vector<HeadMonitorFeedbackWithMetadata> paused_states = paused_state_collection_->pullAllResults(q, false);

  if(paused_states.size() == 0) {
    ROS_WARN_STREAM("No paused states with that time");
    return false;
  } else if(paused_states.size() > 1) {
    ROS_WARN_STREAM("Multiple paused states with time");
    return false;
  }
  paused_state = *paused_states[0];
  return true;
}

bool MoveArmWarehouseLoggerReader::hasPlanningScene(const std::string& hostname,
                                                    const unsigned int id) {
  mongo_ros::Query q = makeQueryForPlanningSceneId(id);  
  q.append("hostname", hostname);
  
  std::vector<PlanningSceneWithMetadata> planning_scenes = planning_scene_collection_->pullAllResults(q, true);
  if(planning_scenes.size() > 0) return true;
  return false;
}

bool MoveArmWarehouseLoggerReader::removePlanningSceneAndAssociatedDataFromWarehouse(const std::string& hostname,
                                                                                     const unsigned int id) {
  mongo_ros::Query q = makeQueryForPlanningSceneId(id);  
  q.append("hostname", hostname);

  unsigned int rem = planning_scene_collection_->removeMessages(q);
  ROS_DEBUG_STREAM("Removed " << rem << " planning scenes");

  bool has_planning_scene = (rem > 0);
  
  rem = motion_plan_request_collection_->removeMessages(q);
  ROS_DEBUG_STREAM("Removed " << rem << " motion plan requests");

  rem = trajectory_collection_->removeMessages(q);
  ROS_DEBUG_STREAM("Removed " << rem << " trajectories");

  rem = outcome_collection_->removeMessages(q);
  ROS_DEBUG_STREAM("Removed " << rem << " outcomes");

  rem = paused_state_collection_->removeMessages(q);
  ROS_DEBUG_STREAM("Removed " << rem << " paused states");

  return has_planning_scene;
}

std::string MoveArmWarehouseLoggerReader::jointTrajectoryToString(const trajectory_msgs::JointTrajectory& trajectory)
{
  std::stringstream returnval;
  returnval << trajectory.points.size() << ",";
  for( unsigned int i=0; i<trajectory.points.size(); i++)
  {
    const trajectory_msgs::JointTrajectoryPoint& point = trajectory.points[i];
    returnval << point.positions.size() << ",";
    for( unsigned int j=0; j<point.positions.size(); j++ )
    {
      returnval << point.positions[j] << ",";
      returnval << point.velocities[j] << ",";
    }
  }

  return returnval.str();
}

void MoveArmWarehouseLoggerReader::stringToJointTrajectory(const std::string& trajectory, trajectory_msgs::JointTrajectory& joint_trajectory_error)
{
  std::stringstream stream(trajectory);
  double position;
  double velocity;
  char c;
  int tsize;
  int psize;
  joint_trajectory_error.points.clear();


  stream >> tsize;
  stream >> c;
  for( int i=0; i<tsize; i++)
  {
    trajectory_msgs::JointTrajectoryPoint point;
    stream >> psize;
    stream >> c;
    for( int j=0; j<psize; j++)
    {
      stream >> position;
      stream >> c;
      stream >> velocity;
      stream >> c;
      point.positions.push_back(position);
      point.velocities.push_back(position);
    }
    joint_trajectory_error.points.push_back(point);
  }
}

// void MoveArmWarehouseLoggerReader::deleteMotionPlanRequestFromWarehouse(const arm_navigation_msgs::PlanningScene& planning_scene,
//                                                                         const std::string& stage_name,
//                                                                         const std::string& ID)
// {
//   mongo_ros::Query q = makeQueryForPlanningSceneTime(time);  
//   q.append("stage_name", stage_name);
//   q.append("motion_request_id", ID);
//   std::vector<MotionPlanRequestWithMetadata> motion_plan_requests = motion_plan_request_collection_->pullAllResults(q, false);
//   coll.removeMessages(q);
//   std::vector<MotionPlanRequestWithMetadata> motion_plan_requests_after = motion_plan_request_collection_->pullAllResults(q, false);
//   ROS_DEBUG_STREAM("Removing " << motion_plan_requests.size()-motion_plan_requests_after.size() << " motion plan requests");
// }
