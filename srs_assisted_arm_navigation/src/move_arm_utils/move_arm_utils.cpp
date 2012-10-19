/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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
 *
 *********************************************************************/

/* \author: Matthew Klingensmith, E. Gil Jones */

//#include <move_arm_warehouse/move_arm_utils.h>
#include "srs_assisted_arm_navigation/move_arm_utils/move_arm_utils.h"
#include <assert.h>
#include <geometric_shapes/shape_operations.h>
#include <planning_environment/util/construct_object.h>

using namespace std;
using namespace arm_navigation_msgs;
using namespace planning_scene_utils;
using namespace collision_space;
using namespace kinematics_msgs;
using namespace arm_navigation_msgs;
using namespace head_monitor_msgs;
using namespace move_arm_warehouse;
using namespace planning_environment;
using namespace planning_models;
using namespace std_msgs;
using namespace trajectory_msgs;
using namespace visualization_msgs;
using namespace arm_navigation_msgs;
using namespace actionlib;
using namespace control_msgs;
using namespace interactive_markers;

//#define MARKER_REFRESH_TIME 0.05
#define MARKER_REFRESH_TIME 0.5
#define SAFE_DELETE(x) if(x != NULL) { delete x; x = NULL; }
#define NOT_MOVING_VELOCITY_THRESHOLD 0.005
#define NOT_MOVING_TIME_THRESHOLD 0.5	//seconds

std_msgs::ColorRGBA makeRandomColor(float brightness, float alpha)
{
  std_msgs::ColorRGBA toReturn;
  toReturn.a = alpha;

  toReturn.r = ((float)(random()) / (float)RAND_MAX) * (1.0f - brightness) + brightness;
  toReturn.g = ((float)(random()) / (float)RAND_MAX) * (1.0f - brightness) + brightness;
  toReturn.b = ((float)(random()) / (float)RAND_MAX) * (1.0f - brightness) + brightness;

  toReturn.r = min(toReturn.r, 1.0f);
  toReturn.g = min(toReturn.g, 1.0f);
  toReturn.b = min(toReturn.b, 1.0f);

  return toReturn;
}

///////////////////////
// PLANNING SCENE DATA
//////////////////////

PlanningSceneData::PlanningSceneData()
{
  setId(0);
  setTimeStamp(ros::Time(ros::WallTime::now().toSec()));
}

PlanningSceneData::PlanningSceneData(unsigned int id, const ros::Time& timestamp, const PlanningScene& scene)
{
  setId(id);
  setPlanningScene(scene);
  setTimeStamp(timestamp);
}

void PlanningSceneData::getRobotState(KinematicState* state)
{
  // Actually converts a robot state message to a kinematic state message (possibly very slow)
  setRobotStateAndComputeTransforms(getPlanningScene().robot_state, *state);
}

////////////////////
//TRAJECTORY DATA
///////////////////

TrajectoryData::TrajectoryData()
{
  setCurrentState(NULL);
  setSource("");
  setGroupName("");
  setColor(makeRandomColor(0.3f, 0.6f));
  reset();
  setId(0);
  showCollisions();
  should_refresh_colors_ = false;
  has_refreshed_colors_ = true;
  refresh_timer_ = ros::Duration(0.0);
  trajectory_error_code_.val = 0;
  setRenderType(CollisionMesh);
  trajectory_render_type_ = Kinematic;
}

TrajectoryData::TrajectoryData(const unsigned int& id, const string& source, const string& groupName,
                  const JointTrajectory& trajectory)
{
  setCurrentState(NULL);
  setId(id);
  setSource(source);
  setGroupName(groupName);
  setTrajectory(trajectory);
  setColor(makeRandomColor(0.3f, 0.6f));
  reset();
  showCollisions();
  should_refresh_colors_ = false;
  has_refreshed_colors_ = true;
  refresh_timer_ = ros::Duration(0.0);
  trajectory_error_code_.val = 0;
  setRenderType(CollisionMesh);
  trajectory_render_type_ = Kinematic;
}

TrajectoryData::TrajectoryData(const unsigned int& id, const string& source, const string& groupName,
                  const JointTrajectory& trajectory, const trajectory_msgs::JointTrajectory& trajectory_error)
{
  setCurrentState(NULL);
  setId(id);
  setSource(source);
  setGroupName(groupName);
  setTrajectory(trajectory);
  setTrajectoryError(trajectory_error);
  setColor(makeRandomColor(0.3f, 0.6f));
  reset();
  showCollisions();
  should_refresh_colors_ = false;
  has_refreshed_colors_ = true;
  refresh_timer_ = ros::Duration(0.0);
  trajectory_error_code_.val = 0;
  setRenderType(CollisionMesh);
  trajectory_render_type_ = Kinematic;
}

void TrajectoryData::advanceToNextClosestPoint(ros::Time time)
{
  unsigned int tsize = getTrajectorySize();

  if(tsize == 0 || getCurrentState() == NULL)
  {
    return;
  }

  unsigned int current_point = getCurrentPoint();
  unsigned int best_point = current_point;
  ros::Duration playback_time_from_start = time - playback_start_time_;

  // Assume strictly increasing timestamps
  // Does not seach through all points, only the points after the current_point.
  for( unsigned int point_index=current_point; point_index<tsize; point_index++ )
  {
    if( trajectory_.points[point_index].time_from_start <= playback_time_from_start )
    {
      best_point = point_index;
    }
    else
    {
      break;
    }
  }
  
  if( best_point != getCurrentPoint() )
  {
    setCurrentPoint((int)best_point);
  }

  if( getCurrentPoint() >= tsize - 1)
  {
   setCurrentPoint(tsize-1);
   stop();
  }

  updateCurrentState();
}

void TrajectoryData::advanceThroughTrajectory(int step)
{
  unsigned int tsize = getTrajectorySize();

  if(tsize == 0 || getCurrentState() == NULL)
  {
    return;
  }

  // Possibly negative point
  if((int)getCurrentPoint() + step < 0)
  {
    setCurrentPoint(0);
  }
  else
  {
    setCurrentPoint((int)getCurrentPoint() + step);
  }

  // Possibly overstepped the end of the trajectory.
  if(getCurrentPoint() >= tsize - 1)
  {
    setCurrentPoint(tsize - 1);
    stop();
  }

  // Create new kinematic state.
  updateCurrentState();
}

void TrajectoryData::updateCurrentState()
{
  if(getTrajectory().points.size() <= 0)
  {
    return;
  }

  map<string, double> joint_values;
  for(unsigned int i = 0; i < getTrajectory().joint_names.size(); i++)
  {
    joint_values[getTrajectory().joint_names[i]] = getTrajectory().points[getCurrentPoint()].positions[i];
  }

  getCurrentState()->setKinematicState(joint_values);
}

void TrajectoryData::updateCollisionMarkers(CollisionModels* cm_, MotionPlanRequestData& motionPlanRequest,
                                            ros::ServiceClient* distance_state_validity_service_client_)
{
  if(areCollisionsVisible())
  {
    const KinematicState* state = getCurrentState();
    collision_markers_.markers.clear();
    if(state == NULL)
    {
      return;
    }
    std_msgs::ColorRGBA bad_color;
    bad_color.a = 1.0f;
    bad_color.r = 1.0f;
    bad_color.g = 0.0f;
    bad_color.b = 0.0f;

    collision_space::EnvironmentModel::AllowedCollisionMatrix acm = cm_->getCurrentAllowedCollisionMatrix();
    cm_->disableCollisionsForNonUpdatedLinks(getGroupName());
    // Get all collisions as little red spheres.
    cm_->getAllCollisionPointMarkers(*state, collision_markers_, bad_color, ros::Duration(MARKER_REFRESH_TIME));

    const KinematicState::JointStateGroup* jsg = state->getJointStateGroup(getGroupName());
    Constraints empty_constraints;
    ArmNavigationErrorCodes code;
    Constraints path_constraints;
    if(motionPlanRequest.hasPathConstraints()) {
      path_constraints = motionPlanRequest.getMotionPlanRequest().path_constraints;
    }
    
    // Update validity of the current state.
    cm_->isKinematicStateValid(*state, jsg->getJointNames(), code, empty_constraints,
                               path_constraints, true);

    cm_->setAlteredAllowedCollisionMatrix(acm);
    GetStateValidity::Request val_req;
    GetStateValidity::Response val_res;
    convertKinematicStateToRobotState(*state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                      val_req.robot_state);

    if(distance_state_validity_service_client_ != NULL) {
      if(!distance_state_validity_service_client_->call(val_req, val_res))
      {
        ROS_INFO_STREAM("Something wrong with distance server");
      }
    }
  }
}




//////////////////////////////
// MOTION PLAN REQUEST DATA
////////////////////////////

MotionPlanRequestData::MotionPlanRequestData(const KinematicState* robot_state)
{
  setSource("");
  setStartColor(makeRandomColor(0.3f, 0.6f));
  setGoalColor(makeRandomColor(0.3f, 0.6f));
  setStartEditable(true);
  setGoalEditable(true);
  setHasGoodIKSolution(true, StartPosition);
  setHasGoodIKSolution(true, GoalPosition);
  setId(0);
  setPathConstraints(false);
  setConstrainRoll(false);
  setConstrainPitch(false);
  setConstrainYaw(false);
  setRollTolerance(.05);
  setPitchTolerance(.05);
  setYawTolerance(.05);
  name_ = "";
  show();
  showCollisions();
  // Note: these must be registered as StateRegistry entries after this request has been created.
  start_state_ = new KinematicState(*robot_state);
  goal_state_ = new KinematicState(*robot_state);
  should_refresh_colors_ = false;
  has_refreshed_colors_ = true;
  refresh_timer_ = ros::Duration(0.0);
  are_joint_controls_visible_ = false;
  setRenderType(CollisionMesh);
}

MotionPlanRequestData::MotionPlanRequestData(const unsigned int& id, const string& source, const MotionPlanRequest& request,
                                             const KinematicState* robot_state,
                                             const std::string& end_effector_name)
{
  // Note: these must be registered as StateRegistry entries after this request has been created.
  start_state_ = new KinematicState(*robot_state);
  goal_state_ = new KinematicState(*robot_state);

  setId(id);
  setSource(source);
  setEndEffectorLink(end_effector_name);
  setMotionPlanRequest(request);

  setStartColor(makeRandomColor(0.3f, 0.6f));
  setGoalColor(makeRandomColor(0.3f, 0.6f));
  setStartEditable(true);
  setGoalEditable(true);
  setHasGoodIKSolution(true, StartPosition);
  setHasGoodIKSolution(true, GoalPosition);

  if(request.path_constraints.orientation_constraints.size() > 0) {
    const OrientationConstraint& oc = request.path_constraints.orientation_constraints[0];
    setPathConstraints(true);
    if(oc.absolute_roll_tolerance < 3.0) {
      setConstrainRoll(true);
      setRollTolerance(oc.absolute_roll_tolerance);
    } else {
      setConstrainRoll(false);
      setRollTolerance(0.05);
    }
    if(oc.absolute_pitch_tolerance < 3.0) {
      setConstrainPitch(true);
      setPitchTolerance(oc.absolute_pitch_tolerance);
    } else {
      setConstrainPitch(false);
      setPitchTolerance(0.05);
    }
    if(oc.absolute_yaw_tolerance < 3.0) {
      setConstrainYaw(true);
      setYawTolerance(oc.absolute_yaw_tolerance);
    } else {
      setConstrainYaw(false);
      setYawTolerance(0.05);
    }
  } else {
    setPathConstraints(false);
    setConstrainRoll(false);
    setConstrainPitch(false);
    setConstrainYaw(false);
    setRollTolerance(.05);
    setPitchTolerance(.05);
    setYawTolerance(.05);
  }
  show();
  showCollisions();

  should_refresh_colors_ = false;
  has_refreshed_colors_ = true;
  refresh_timer_ = ros::Duration(0.0);
  are_joint_controls_visible_ = false;

  setRenderType(CollisionMesh);
}

// Kinematic states must be converted to joint constraint message
void MotionPlanRequestData::updateGoalState()
{
  setRobotStateAndComputeTransforms(getMotionPlanRequest().start_state, *goal_state_);

  vector<JointConstraint>& constraints = getMotionPlanRequest().goal_constraints.joint_constraints;

  map<string, double> jointValues;
  for(size_t i = 0; i < constraints.size(); i++)
  {
    JointConstraint& constraint = constraints[i];
    jointValues[constraint.joint_name] = constraint.position;
  }

  goal_state_->setKinematicState(jointValues);

  if(getMotionPlanRequest().goal_constraints.position_constraints.size() > 0 &&
     getMotionPlanRequest().goal_constraints.orientation_constraints.size() > 0) {
    if(getMotionPlanRequest().goal_constraints.position_constraints[0].link_name != end_effector_link_) {
      ROS_WARN_STREAM("Can't apply position constraints to link " 
                      << getMotionPlanRequest().goal_constraints.position_constraints[0].link_name
                      << " instead of link " << end_effector_link_);
      return;
    }
    ROS_DEBUG_STREAM("Tolerances are " << motion_plan_request_.path_constraints.orientation_constraints[0].absolute_roll_tolerance 
                    << " " << motion_plan_request_.path_constraints.orientation_constraints[0].absolute_pitch_tolerance 
                    << " " << motion_plan_request_.path_constraints.orientation_constraints[0].absolute_yaw_tolerance);

    geometry_msgs::PoseStamped pose = 
      arm_navigation_msgs::poseConstraintsToPoseStamped(getMotionPlanRequest().goal_constraints.position_constraints[0],
                                                        getMotionPlanRequest().goal_constraints.orientation_constraints[0]);
    tf::Transform end_effector_pose = toBulletTransform(pose.pose);
    goal_state_->updateKinematicStateWithLinkAt(end_effector_link_, end_effector_pose);
  }
}

// Kinematic state must be converted to robot state message
void MotionPlanRequestData::updateStartState()
{
  setRobotStateAndComputeTransforms(getMotionPlanRequest().start_state, *start_state_);
}

void MotionPlanRequestData::setJointControlsVisible(bool visible, PlanningSceneEditor* editor)
{
  are_joint_controls_visible_ = visible;

  if(visible)
  {
    if(isStartVisible() && isStartEditable())
    {
      editor->createJointMarkers(*this, StartPosition);
    }
    else
    {
      editor->deleteJointMarkers(*this, StartPosition);
    }
    if(isEndVisible() && isGoalEditable())
    {
      editor->createJointMarkers(*this, GoalPosition);
    }
    else
    {
      editor->deleteJointMarkers(*this, GoalPosition);
    }
  }
  else
  {
    editor->deleteJointMarkers(*this, StartPosition);
    editor->deleteJointMarkers(*this, GoalPosition);
  }
}

void MotionPlanRequestData::setStartStateValues(std::map<std::string, double>& joint_values)
{
  setStateChanged(true);
  start_state_->setKinematicState(joint_values);
}

void MotionPlanRequestData::setGoalStateValues(std::map<std::string, double>& joint_values)
{
  setStateChanged(true);
  goal_state_->setKinematicState(joint_values);

  getMotionPlanRequest().goal_constraints.joint_constraints.resize(joint_values.size());
  int constraints = 0;
  for(std::map<std::string, double>::iterator it = joint_values.begin(); it != joint_values.end(); it++, constraints++)
  {
    getMotionPlanRequest().goal_constraints.joint_constraints[constraints].joint_name = it->first;
    getMotionPlanRequest().goal_constraints.joint_constraints[constraints].position = it->second;
    getMotionPlanRequest().goal_constraints.joint_constraints[constraints].tolerance_above = 0.001;
    getMotionPlanRequest().goal_constraints.joint_constraints[constraints].tolerance_below = 0.001;
  }

  if(getMotionPlanRequest().goal_constraints.position_constraints.size() > 0 &&
     getMotionPlanRequest().goal_constraints.orientation_constraints.size() > 0) {
    if(getMotionPlanRequest().goal_constraints.position_constraints[0].link_name != end_effector_link_) {
      ROS_WARN_STREAM("Can't apply position constraints to link " 
                      << getMotionPlanRequest().goal_constraints.position_constraints[0].link_name
                      << " instead of link " << end_effector_link_);
      return;
    }
    MotionPlanRequest mpr;
    setGoalAndPathPositionOrientationConstraints(mpr, GoalPosition);
    motion_plan_request_.goal_constraints.position_constraints = mpr.goal_constraints.position_constraints;
    motion_plan_request_.goal_constraints.orientation_constraints = mpr.goal_constraints.orientation_constraints;
    motion_plan_request_.path_constraints.position_constraints = mpr.path_constraints.position_constraints;
    motion_plan_request_.path_constraints.orientation_constraints = mpr.path_constraints.orientation_constraints;

    ROS_DEBUG_STREAM("Tolerances are " << motion_plan_request_.path_constraints.orientation_constraints[0].absolute_roll_tolerance 
                     << " " << motion_plan_request_.path_constraints.orientation_constraints[0].absolute_pitch_tolerance 
                     << " " << motion_plan_request_.path_constraints.orientation_constraints[0].absolute_yaw_tolerance);
  }
}

void MotionPlanRequestData::setGoalAndPathPositionOrientationConstraints(arm_navigation_msgs::MotionPlanRequest& mpr, 
                                                                         planning_scene_utils::PositionType type) const
{
  mpr = motion_plan_request_;

  KinematicState* state = NULL;

  if(type == StartPosition)
  {
    state = start_state_;
  }
  else
  {
    state = goal_state_;
  }

  std::string world_frame = state->getKinematicModel()->getRoot()->getParentFrameId();

  mpr.goal_constraints.joint_constraints.clear();

  mpr.goal_constraints.position_constraints.resize(1);
  mpr.goal_constraints.orientation_constraints.resize(1);    
  geometry_msgs::PoseStamped end_effector_wrist_pose;
  tf::poseTFToMsg(goal_state_->getLinkState(end_effector_link_)->getGlobalLinkTransform(),
                  end_effector_wrist_pose.pose);
  end_effector_wrist_pose.header.frame_id = world_frame;
  arm_navigation_msgs::poseStampedToPositionOrientationConstraints(end_effector_wrist_pose,
                                                                   end_effector_link_,
                                                                   mpr.goal_constraints.position_constraints[0],
                                                                   mpr.goal_constraints.orientation_constraints[0]);
  mpr.path_constraints.orientation_constraints.resize(1);

  arm_navigation_msgs::OrientationConstraint& goal_constraint = mpr.goal_constraints.orientation_constraints[0];
  arm_navigation_msgs::OrientationConstraint& path_constraint = mpr.path_constraints.orientation_constraints[0];

  tf::Transform cur = state->getLinkState(end_effector_link_)->getGlobalLinkTransform();
  //tfScalar roll, pitch, yaw;
  //cur.getBasis().getRPY(roll,pitch,yaw);
  goal_constraint.header.frame_id = world_frame;
  goal_constraint.header.stamp = ros::Time(ros::WallTime::now().toSec());
  goal_constraint.link_name = end_effector_link_;
  tf::quaternionTFToMsg(cur.getRotation(), goal_constraint.orientation);
  goal_constraint.absolute_roll_tolerance = 0.04;
  goal_constraint.absolute_pitch_tolerance = 0.04;
  goal_constraint.absolute_yaw_tolerance = 0.04;

  path_constraint.header.frame_id = world_frame;
  path_constraint.header.stamp = ros::Time(ros::WallTime::now().toSec());
  path_constraint.link_name = end_effector_link_;
  tf::quaternionTFToMsg(cur.getRotation(), path_constraint.orientation);
  path_constraint.type = path_constraint.HEADER_FRAME;
  if(getConstrainRoll()) {
    path_constraint.absolute_roll_tolerance = getRollTolerance();
  } else {
    path_constraint.absolute_roll_tolerance = M_PI;
  }
  if(getConstrainPitch()) {
    path_constraint.absolute_pitch_tolerance = getPitchTolerance();
  } else {
    path_constraint.absolute_pitch_tolerance = M_PI;
  }
  if(getConstrainYaw()) {
    path_constraint.absolute_yaw_tolerance = getYawTolerance();
  } else {
    path_constraint.absolute_yaw_tolerance = M_PI;
  }
}

void MotionPlanRequestData::updateCollisionMarkers(CollisionModels* cm_,
                                                   ros::ServiceClient* distance_state_validity_service_client_)
{
  if(areCollisionsVisible())
  {
    const KinematicState* state = getStartState();
    collision_markers_.markers.clear();
    if(state == NULL)
    {
      return;
    }

    ///////
    // Start state block
    ///////
    std_msgs::ColorRGBA bad_color;
    bad_color.a = 1.0f;
    bad_color.r = 1.0f;
    bad_color.g = 0.0f;
    bad_color.b = 0.0f;
    collision_space::EnvironmentModel::AllowedCollisionMatrix acm = cm_->getCurrentAllowedCollisionMatrix();
    cm_->disableCollisionsForNonUpdatedLinks(getGroupName());
    // Get all the collision points as little red spheres.
    if(isStartVisible()) {
      cm_->getAllCollisionPointMarkers(*state, collision_markers_, bad_color, ros::Duration(MARKER_REFRESH_TIME));
      const KinematicState::JointStateGroup* jsg = state->getJointStateGroup(getGroupName());
      ArmNavigationErrorCodes code;
      Constraints empty_constraints;
      // Ensure that the state is valid.
      Constraints path_constraints;
      if(hasPathConstraints()) {
        path_constraints = getMotionPlanRequest().path_constraints;
      }
      cm_->isKinematicStateValid(*state, jsg->getJointNames(), code, empty_constraints,
                                 path_constraints, true);
      
      GetStateValidity::Request val_req;
      GetStateValidity::Response val_res;
      convertKinematicStateToRobotState(*state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                        val_req.robot_state);
      
      if(distance_state_validity_service_client_ != NULL) {
        if(!distance_state_validity_service_client_->call(val_req, val_res))
        {
          ROS_INFO_STREAM("Something wrong with distance server");
        }
      }
    }
    ////////
    // End State block
    ///////
    if(isEndVisible()) {
      state = getGoalState();
      if(state == NULL)
      {
        return;
      }
      cm_->getAllCollisionPointMarkers(*state, collision_markers_, bad_color, ros::Duration(MARKER_REFRESH_TIME));
      const KinematicState::JointStateGroup* jsg = state->getJointStateGroup(getGroupName());
      ArmNavigationErrorCodes code;
      Constraints empty_constraints;
      Constraints path_constraints;
      if(hasPathConstraints()) {
        path_constraints = getMotionPlanRequest().path_constraints;
      }
      cm_->isKinematicStateValid(*state, jsg->getJointNames(), code, empty_constraints,
                                 path_constraints, true);

      GetStateValidity::Request val_req;
      GetStateValidity::Response val_res;
      convertKinematicStateToRobotState(*state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                        val_req.robot_state);

      if(distance_state_validity_service_client_ != NULL) {
        if(!distance_state_validity_service_client_->call(val_req, val_res))
        {
          ROS_INFO_STREAM("Something wrong with distance server");
        }
      }
    }
    cm_->setAlteredAllowedCollisionMatrix(acm);
  }
}

// Warning: lots of copying going on here.
std::vector<std::string> MotionPlanRequestData::getJointNamesInGoal()
{
  std::vector<JointConstraint>& constraints = getMotionPlanRequest().goal_constraints.joint_constraints;
  std::vector<std::string> toReturn;

  for(size_t i = 0; i < constraints.size(); i++)
  {
    JointConstraint& constraint = constraints[i];
    toReturn.push_back(constraint.joint_name);
  }

  return toReturn;
}

// Warning: calls getJointNamesInGoal
bool MotionPlanRequestData::isJointNameInGoal(std::string joint)
{
  vector<string> joints = getJointNamesInGoal();
  for(size_t i = 0; i < joints.size(); i++)
  {
    if(joints[i] == joint)
    {
      return true;
    }
  }

  return false;
}

////////////////////////////////
// PLANNING SCENE EDITOR
///////////////////////////////

PlanningSceneEditor::PlanningSceneEditor()
{
  setRobotState(NULL, false);
  setCollisionModel(NULL, false);
  interactive_marker_server_ = NULL;
  collision_aware_ik_services_ = NULL;
  non_collision_aware_ik_services_ = NULL;
  interpolated_ik_services_ = NULL;
  selectable_objects_ = NULL;
  ik_controllers_ = NULL;
  max_collision_object_id_ = 0;
  active_planner_index_ = 1;
  use_primary_filter_ = true;
  string robot_description_name = nh_.resolveName("robot_description", true);
  cm_ = new CollisionModels(robot_description_name);
}

PlanningSceneEditor::PlanningSceneEditor(PlanningSceneParameters& params)
{
  ///////
  /// Memory initialization
  /////
  params_ = params;
  monitor_status_ = idle;
  max_collision_object_id_ = 0;
  active_planner_index_ = 1;
  use_primary_filter_ = true;
  last_collision_object_color_.r = 0.7;
  last_collision_object_color_.g = 0.7;
  last_collision_object_color_.b = 0.7;
  last_collision_object_color_.a = 1.0;
  last_mesh_object_color_.r = 0.7;
  last_mesh_object_color_.g = 0.7;
  last_mesh_object_color_.b = 0.7;
  last_mesh_object_color_.a = 1.0;

  collision_aware_ik_services_ = new map<string, ros::ServiceClient*> ();
  non_collision_aware_ik_services_ = new map<string, ros::ServiceClient*> ();
  interpolated_ik_services_ = new map<string, ros::ServiceClient*> ();
  selectable_objects_ = new map<string, SelectableObject> ();
  ik_controllers_ = new map<string, IKController> ();

  string robot_description_name = nh_.resolveName("robot_description", true);
  cm_ = new CollisionModels(robot_description_name);
  robot_state_ = new KinematicState(cm_->getKinematicModel());
  robot_state_->setKinematicStateToDefault();

  ////
  /// Interactive markers
  ////
  interactive_marker_server_
      = new interactive_markers::InteractiveMarkerServer("planning_scene_warehouse_viewer_controls", "", false);

  collision_object_movement_feedback_ptr_
      = boost::bind(&PlanningSceneEditor::collisionObjectMovementCallback, this, _1);
  collision_object_selection_feedback_ptr_ = boost::bind(&PlanningSceneEditor::collisionObjectSelectionCallback, this,
                                                         _1);
  joint_control_feedback_ptr_ = boost::bind(&PlanningSceneEditor::JointControllerCallback, this, _1);
  ik_control_feedback_ptr_ = boost::bind(&PlanningSceneEditor::IKControllerCallback, this, _1);
  attached_collision_object_feedback_ptr_ = boost::bind(&PlanningSceneEditor::attachedCollisionObjectInteractiveCallback, this, _1);

  //////
  /// Publishers
  /////
  joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState> ("joint_states", 10);
  vis_marker_publisher_ = nh_.advertise<visualization_msgs::Marker> (params.vis_topic_name_, 128);
  vis_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray> (params.vis_topic_name_ + "_array", 128);
  move_arm_warehouse_logger_reader_ = new MoveArmWarehouseLoggerReader();


  /////
  /// Subscribers
  //////
  // if(params.sync_robot_state_with_gazebo_)
  // {
  //   ros::service::waitForService("/gazebo/set_model_configuration");
  //   ros::service::waitForService(params.list_controllers_service_);
  //   ros::service::waitForService(params.load_controllers_service_);
  //   ros::service::waitForService(params.unload_controllers_service_);
  //   ros::service::waitForService(params.switch_controllers_service_);
  //   ros::service::waitForService("/gazebo/pause_physics");
  //   ros::service::waitForService("/gazebo/unpause_physics");
  //   ros::service::waitForService("/gazebo/set_link_properties");
  //   ros::service::waitForService("/gazebo/get_link_properties");
  // }

  if(params.left_arm_group_ != "none")
  {
    ros::service::waitForService(params.left_ik_name_);
  }

  if(params.right_arm_group_ != "none")
  {
    ros::service::waitForService(params.right_ik_name_);
  }

  if(params.planner_1_service_name_ != "none")
  {
    ros::service::waitForService(params.planner_1_service_name_);
  }

  if(params.proximity_space_service_name_ != "none")
  {
    ros::service::waitForService(params.proximity_space_service_name_);
  }

  if(params.proximity_space_validity_name_ != "none")
  {
    ros::service::waitForService(params.proximity_space_validity_name_);
  }

  // if(params.sync_robot_state_with_gazebo_)
  // {
  //   gazebo_joint_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration", true);
  //   list_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::ListControllers>(params.list_controllers_service_, true);
  //   load_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::LoadController>(params.load_controllers_service_, true);
  //   unload_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::UnloadController>(params.unload_controllers_service_, true);
  //   switch_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::SwitchController>(params.switch_controllers_service_, true);
  //   pause_gazebo_client_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics", true);
  //   unpause_gazebo_client_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics", true);
  //   set_link_properties_client_ = nh_.serviceClient<gazebo_msgs::SetLinkProperties>("/gazebo/set_link_properties", true);
  //   get_link_properties_client_ = nh_.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties", true);
  // }

  if(params.left_arm_group_ != "none")
  {
    left_ik_service_client_ = nh_.serviceClient<GetConstraintAwarePositionIK> (params.left_ik_name_, true);
    non_coll_left_ik_service_client_ = nh_.serviceClient<GetPositionIK> (params.non_coll_left_ik_name_, true);
  }
  if(params.right_arm_group_ != "none")
  {
    right_ik_service_client_ = nh_.serviceClient<GetConstraintAwarePositionIK> (params.right_ik_name_, true);
    non_coll_right_ik_service_client_ = nh_.serviceClient<GetPositionIK> (params.non_coll_right_ik_name_, true);
  }

  if(params.planner_1_service_name_ != "none")
  {
    planning_1_service_client_ = nh_.serviceClient<GetMotionPlan> (params.planner_1_service_name_, true);
  }

  if(params.planner_2_service_name_ != "none")
  {
    planning_2_service_client_ = nh_.serviceClient<GetMotionPlan> (params.planner_2_service_name_, true);
  }

  if(params.trajectory_filter_1_service_name_ != "none")
  {
    trajectory_filter_1_service_client_
        = nh_.serviceClient<FilterJointTrajectoryWithConstraints> (params.trajectory_filter_1_service_name_);
  }

  if(params.trajectory_filter_2_service_name_ != "none")
  {
    trajectory_filter_2_service_client_
        = nh_.serviceClient<FilterJointTrajectoryWithConstraints> (params.trajectory_filter_2_service_name_);
  }

  if(params.proximity_space_service_name_ != "none")
  {
    distance_aware_service_client_ = nh_.serviceClient<GetMotionPlan> (params.proximity_space_service_name_, true);
  }

  if(params.proximity_space_validity_name_ != "none")
  {
    distance_state_validity_service_client_
        = nh_.serviceClient<GetStateValidity> (params.proximity_space_validity_name_, true);
  }

  if(params.proximity_space_planner_name_ != "none")
  {
    collision_proximity_planner_client_ = nh_.serviceClient<GetMotionPlan> (params.proximity_space_planner_name_, true);
  }

  set_planning_scene_diff_client_ = nh_.serviceClient<SetPlanningSceneDiff> (params.set_planning_scene_diff_name_);

  if(params.use_robot_data_)
  {
    if(params_.right_arm_group_ != "none")
    {
      arm_controller_map_[params_.right_arm_group_] = new actionlib::SimpleActionClient<
          control_msgs::FollowJointTrajectoryAction>(params.execute_right_trajectory_, true);

      while(ros::ok() && !arm_controller_map_[params_.right_arm_group_]->waitForServer(ros::Duration(1.0)))
      {
        ROS_INFO("Waiting for the right_joint_trajectory_action server to come up.");
      }
    }

    if(params.left_arm_group_ != "none")
    {
      arm_controller_map_[params.left_arm_group_] = new actionlib::SimpleActionClient<
          control_msgs::FollowJointTrajectoryAction>(params.execute_left_trajectory_, true);

      while(ros::ok() && !arm_controller_map_[params.left_arm_group_]->waitForServer(ros::Duration(1.0)))
      {
        ROS_INFO("Waiting for the left_joint_trajectory_action server to come up.");
      }
    }
  }

  if(params.left_arm_group_ != "none")
  {
    (*collision_aware_ik_services_)[params.left_ik_link_] = &left_ik_service_client_;
  }

  if(params.right_arm_group_ != "none")
  {
    (*collision_aware_ik_services_)[params.right_ik_link_] = &right_ik_service_client_;
  }

  if(params.left_arm_group_ != "none")
  {
    (*non_collision_aware_ik_services_)[params.left_ik_link_] = &non_coll_left_ik_service_client_;
  }

  if(params.left_arm_group_ != "none")
  {
    (*non_collision_aware_ik_services_)[params.right_ik_link_] = &non_coll_right_ik_service_client_;
  }

  if(params.right_interpolate_service_name_ != "none")
  {
    while(ros::ok() && !ros::service::waitForService(params.right_interpolate_service_name_, ros::Duration(1.0)))
    {
      ROS_INFO_STREAM("Waiting for the right interpolation server to come up: " << params.right_interpolate_service_name_);
    }
    right_interpolate_service_client_ = nh_.serviceClient<GetMotionPlan> (params.right_interpolate_service_name_, true);
    (*interpolated_ik_services_)[params.right_ik_link_] = &right_interpolate_service_client_;
  }
  if(params.left_interpolate_service_name_ != "none")
  {
    while(ros::ok() && !ros::service::waitForService(params.left_interpolate_service_name_, ros::Duration(1.0)))
    {
      ROS_INFO("Waiting for the left interpolation server to come up.");
    }
    left_interpolate_service_client_ = nh_.serviceClient<GetMotionPlan> (params.left_interpolate_service_name_, true);
    (*interpolated_ik_services_)[params.left_ik_link_] = &left_interpolate_service_client_;
  }

  /////
  /// Interactive menus
  /////
  menu_entry_maps_["Collision Object"] = MenuEntryMap();
  menu_entry_maps_["Collision Object Selection"] = MenuEntryMap();
  menu_entry_maps_["IK Control"] = MenuEntryMap();
  registerMenuEntry("Collision Object Selection", "Delete", collision_object_selection_feedback_ptr_);
  registerMenuEntry("Collision Object Selection", "Select", collision_object_selection_feedback_ptr_);
  registerMenuEntry("Collision Object", "Delete", collision_object_movement_feedback_ptr_);
  registerMenuEntry("Collision Object", "Deselect", collision_object_movement_feedback_ptr_);
  registerMenuEntry("Collision Object", "Attach", collision_object_movement_feedback_ptr_);
  registerMenuEntry("Attached Collision Object", "Detach", attached_collision_object_feedback_ptr_);
  MenuHandler::EntryHandle resize_mode_entry = registerMenuEntry("Collision Object", "Resize Mode", collision_object_movement_feedback_ptr_);
  MenuHandler::EntryHandle off = menu_handler_map_["Collision Object"].insert(resize_mode_entry, "Off", collision_object_movement_feedback_ptr_);
  menu_entry_maps_["Collision Object"]["Off"] = off;
  menu_handler_map_["Collision Object"].setCheckState(off, MenuHandler::CHECKED);
  last_resize_handle_ = off;
  MenuHandler::EntryHandle grow = menu_handler_map_["Collision Object"].insert(resize_mode_entry, "Grow", collision_object_movement_feedback_ptr_);
  menu_entry_maps_["Collision Object"]["Grow"] = grow;
  menu_handler_map_["Collision Object"].setCheckState(grow, MenuHandler::UNCHECKED);
  MenuHandler::EntryHandle shrink = menu_handler_map_["Collision Object"].insert(resize_mode_entry, "Shrink", collision_object_movement_feedback_ptr_);
  menu_handler_map_["Collision Object"].setCheckState(shrink, MenuHandler::UNCHECKED);
  menu_entry_maps_["Collision Object"]["Shrink"] = shrink;
  registerMenuEntry("IK Control", "Map to Robot State", ik_control_feedback_ptr_);
  registerMenuEntry("IK Control", "Map from Robot State", ik_control_feedback_ptr_);
  registerMenuEntry("IK Control", "Map to Other Orientation", ik_control_feedback_ptr_);
  registerMenuEntry("IK Control", "Go To Last Good State", ik_control_feedback_ptr_);
  registerMenuEntry("IK Control", "Randomly Perturb", ik_control_feedback_ptr_);
  registerMenuEntry("IK Control", "Plan New Trajectory", ik_control_feedback_ptr_);
  registerMenuEntry("IK Control", "Filter Last Trajectory", ik_control_feedback_ptr_);
  registerMenuEntry("IK Control", "Delete Request", ik_control_feedback_ptr_);

  if(params_.use_robot_data_)
  {
    registerMenuEntry("IK Control", "Execute Last Trajectory", ik_control_feedback_ptr_);
  }

  /////
  /// Connection with sim data
  /////
  if(params.use_robot_data_)
  {
    joint_state_subscriber_ = nh_.subscribe("joint_states", 25, &PlanningSceneEditor::jointStateCallback, this);
  }

  /////
  /// Connection with joint controller
  if(params.use_robot_data_)
  {
    r_arm_controller_state_subscriber_ = nh_.subscribe("r_arm_controller/state", 25, &PlanningSceneEditor::jointTrajectoryControllerStateCallback, this);
    l_arm_controller_state_subscriber_ = nh_.subscribe("l_arm_controller/state", 25, &PlanningSceneEditor::jointTrajectoryControllerStateCallback, this);
  }
}

void PlanningSceneEditor::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
  if(robot_state_ == NULL) return;

  std::map<std::string, double> joint_state_map;
  std::map<std::string, double> joint_velocity_map;
  
  //message already been validated in kmsm
  for(unsigned int i = 0; i < joint_state->position.size(); ++i)
  {
    joint_state_map[joint_state->name[i]] = joint_state->position[i];
    joint_velocity_map[joint_state->name[i]] = joint_state->velocity[i];
  }
  //getting base transform
  lockScene();
  std::vector<planning_models::KinematicState::JointState*>& joint_state_vector = robot_state_->getJointStateVector();  
  for(std::vector<planning_models::KinematicState::JointState*>::iterator it = joint_state_vector.begin();
      it != joint_state_vector.end();
      it++) {
    bool tfSets = false;
    //see if we need to update any transforms
    std::string parent_frame_id = (*it)->getParentFrameId();
    std::string child_frame_id = (*it)->getChildFrameId();
    if(!parent_frame_id.empty() && !child_frame_id.empty()) {
      std::string err;
      ros::Time tm;
      tf::StampedTransform transf;
      bool ok = false;
      if (transform_listener_.getLatestCommonTime(parent_frame_id, child_frame_id, tm, &err) == tf::NO_ERROR) {
        ok = true;
        try
        {
          transform_listener_.lookupTransform(parent_frame_id, child_frame_id, tm, transf);
        }
        catch(tf::TransformException& ex)
        {
          ROS_ERROR("Unable to lookup transform from %s to %s.  Exception: %s", parent_frame_id.c_str(), child_frame_id.c_str(), ex.what());
          ok = false;
        }
      } else {
        ROS_DEBUG("Unable to lookup transform from %s to %s: no common time.", parent_frame_id.c_str(), child_frame_id.c_str());
        ok = false;
      }
      if(ok) {
        tfSets = (*it)->setJointStateValues(transf);
      }
    }
    (*it)->setJointStateValues(joint_state_map);
  }
  robot_state_->updateKinematicLinks();
  robot_state_->getKinematicStateValues(robot_state_joint_values_);
  unlockScene();
}

void PlanningSceneEditor::jointTrajectoryControllerStateCallback(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr& joint_controller_state)
{
  trajectory_msgs::JointTrajectoryPoint actual = joint_controller_state->actual;
  trajectory_msgs::JointTrajectoryPoint error = joint_controller_state->error;
  bool robot_stopped = true;

  // Records trajectory if currently executing.
  if(monitor_status_ == Executing || monitor_status_ == WaitingForStop)
  {
    // Filter out the joints of the other group/arm
    if( logged_trajectory_.joint_names[0] != joint_controller_state->joint_names[0] )
    {
      return;
    }

    trajectory_msgs::JointTrajectoryPoint point;
    trajectory_msgs::JointTrajectoryPoint error_point;

    // note-- could record accelerations as well, if we wanted to.
    unsigned int num_joints = logged_trajectory_.joint_names.size();
    point.positions.resize(num_joints);
    point.velocities.resize(num_joints);
    error_point.positions.resize(num_joints);
    error_point.velocities.resize(num_joints);

    for(unsigned int i = 0; i < num_joints; i++)
    {
      point.positions[i] = actual.positions[i];
      point.velocities[i] = actual.velocities[i];
      error_point.positions[i] = error.positions[i];
      error_point.velocities[i] = error.velocities[i];
    }

    ros::Duration time_from_start = ros::Time(ros::Time::now().toSec()) - logged_trajectory_start_time_;
    point.time_from_start = time_from_start;
    error_point.time_from_start = time_from_start;

    logged_trajectory_.points.push_back(point);
    logged_trajectory_controller_error_.points.push_back(error_point);

    // Stop recording if the robot has stopped for a period of time.
    if(monitor_status_ == WaitingForStop )
    {
      for(unsigned int i = 0; i < num_joints; i++)
      {
        if( point.velocities[i] > NOT_MOVING_VELOCITY_THRESHOLD )
        {
          robot_stopped = false;
        }
      }

      if( robot_stopped )
      {
        if( (ros::Time::now()-time_of_last_moving_notification_).toSec() >= NOT_MOVING_TIME_THRESHOLD )
        {
          armHasStoppedMoving();
        }
      }
      else
      {
        time_of_last_moving_notification_ = ros::Time::now();
      }
    }
  }
}

PlanningSceneEditor::~PlanningSceneEditor()
{
  SAFE_DELETE(robot_state_);
  SAFE_DELETE(interactive_marker_server_);
  SAFE_DELETE(selectable_objects_);
  SAFE_DELETE(ik_controllers_);
  SAFE_DELETE(collision_aware_ik_services_);
  SAFE_DELETE(non_collision_aware_ik_services_);
  SAFE_DELETE(interpolated_ik_services_);
}

void PlanningSceneEditor::makeSelectableAttachedObjectFromPlanningScene(const arm_navigation_msgs::PlanningScene& scene,
                                                                        arm_navigation_msgs::AttachedCollisionObject& att)
{
  std_msgs::ColorRGBA color;
  color.r = 0.5;
  color.g = 0.5;
  color.b = 0.5;
  color.a = 1.0;
  
  //need to get this in the right frame for adding the collision object
  //intentionally copying object
  arm_navigation_msgs::CollisionObject coll = att.object;
  {
    planning_models::KinematicState state(cm_->getKinematicModel());
    planning_environment::setRobotStateAndComputeTransforms(scene.robot_state,
                                                            state);
    geometry_msgs::PoseStamped ret_pose;
    cm_->convertPoseGivenWorldTransform(*robot_state_,
                                        cm_->getWorldFrameId(),
                                        coll.header,
                                        coll.poses[0],
                                        ret_pose);
    coll.header = ret_pose.header;
    coll.poses[0] = ret_pose.pose;
  }
  createSelectableMarkerFromCollisionObject(coll, coll.id, "", color);
  attachCollisionObject(att.object.id,
                        att.link_name,
                        att.touch_links);
  changeToAttached(att.object.id);
}

void PlanningSceneEditor::setCurrentPlanningScene(std::string planning_scene_name, bool loadRequests, bool loadTrajectories)
{
  if(planning_scene_map_.find(planning_scene_name) == planning_scene_map_.end()) {
    ROS_INFO_STREAM("Haven't got a planning scene for name " << planning_scene_name << " so can't set");
    return;
  }

  lockScene();

  // Need to do this to clear old scene state.
  deleteKinematicStates();

  if(planning_scene_name == "")
  {
    ROS_INFO_STREAM("No new scene");
    current_planning_scene_name_ = planning_scene_name;
    unlockScene();
    return;
  }

  /////
  /// Get rid of old interactive markers.
  //////
  for(map<string, SelectableObject>::iterator it = selectable_objects_->begin(); it != selectable_objects_->end(); it++)
  {
    interactive_marker_server_->erase(it->second.selection_marker_.name);
    interactive_marker_server_->erase(it->second.control_marker_.name);
  }
  selectable_objects_->clear();

  for(map<string, IKController>::iterator it = (*ik_controllers_).begin(); it != (*ik_controllers_).end(); it++)
  {
    interactive_marker_server_->erase(it->second.end_controller_.name);
    interactive_marker_server_->erase(it->second.start_controller_.name);
  }
  interactive_marker_server_->applyChanges();

  (*ik_controllers_).clear();


  std::vector<unsigned int> mprDeletions;
  /////
  /// Make sure all old trajectories and MPRs are gone.
  /////
  for(map<string, MotionPlanRequestData>::iterator it = motion_plan_map_.begin(); it != motion_plan_map_.end(); it ++)
  {
    mprDeletions.push_back(it->second.getId());
  }

  std::vector<unsigned int> erased_trajectories;
  for(size_t i = 0; i < mprDeletions.size(); i++)
  {
    deleteMotionPlanRequest(mprDeletions[i], erased_trajectories);
  } 

  motion_plan_map_.clear();

  if(!trajectory_map_.empty()) {
    ROS_INFO_STREAM("Some trajectories orphaned");
  }

  /////
  /// Load planning scene
  /////
  current_planning_scene_name_ = planning_scene_name;
  PlanningSceneData& scene = planning_scene_map_[planning_scene_name];
  error_map_.clear();
  scene.getPipelineStages().clear();
  scene.getErrorCodes().clear();
  getPlanningSceneOutcomes(scene.getId(), scene.getPipelineStages(), scene.getErrorCodes(), error_map_);
  
  /////
  /// Create collision object.
  /////
  for(size_t i = 0; i < scene.getPlanningScene().collision_objects.size(); i++)
  {
    //otherwise might be associated with an attached collision object
    std_msgs::ColorRGBA color;
    color.r = 0.5;
    color.g = 0.5;
    color.b = 0.5;
    color.a = 1.0;
    createSelectableMarkerFromCollisionObject(scene.getPlanningScene().collision_objects[i], scene.getPlanningScene().collision_objects[i].id, scene.getPlanningScene().collision_objects[i].id, color);
  }

  /////
  /// Create collision object.
  /////
  for(size_t i = 0; i < scene.getPlanningScene().attached_collision_objects.size(); i++)
  {
    makeSelectableAttachedObjectFromPlanningScene(scene.getPlanningScene(),
                                                  scene.getPlanningScene().attached_collision_objects[i]);
  }
  
  /////
  /// Load motion plan requests
  /////
  if(loadRequests)
  {
    vector<unsigned int> ids;
    vector<string> stageNames;
    vector<MotionPlanRequest> requests;
    move_arm_warehouse_logger_reader_->getAssociatedMotionPlanRequests("", scene.getId(), ids, stageNames, requests);
    unsigned int planning_scene_id = getPlanningSceneIdFromName(planning_scene_name);
    initMotionPlanRequestData(planning_scene_id, ids, stageNames, requests);
    
    for(size_t j = 0; j < ids.size(); j++)
    {
      MotionPlanRequest req;
      unsigned int motion_id = ids[j];

      MotionPlanRequestData& motion_data = motion_plan_map_[getMotionPlanRequestNameFromId(motion_id)];
      motion_data.setPlanningSceneId(planning_scene_id);

      std::vector<JointTrajectory> trajs;
      std::vector<JointTrajectory> traj_cnt_errs;
      std::vector<string> sources;
      std::vector<unsigned int> traj_ids;
      std::vector<ros::Duration> durations;
      std::vector<int32_t> errors;
      
      /////
      /// Load trajectories
      /////
      if(loadTrajectories)
      {
        move_arm_warehouse_logger_reader_->getAssociatedJointTrajectories("", scene.getId(), motion_id, trajs, traj_cnt_errs, sources,
                                                                          traj_ids, durations, errors);

        for(size_t k = 0; k < trajs.size(); k++)
        {
          TrajectoryData trajectory_data;
          trajectory_data.setTrajectory(trajs[k]);
          trajectory_data.setTrajectoryError(traj_cnt_errs[k]);
          trajectory_data.setSource(sources[k]);
          trajectory_data.setId(traj_ids[k]);
          trajectory_data.setMotionPlanRequestId(motion_data.getId());
          trajectory_data.setPlanningSceneId(planning_scene_id);
          trajectory_data.setVisible(true);
          trajectory_data.setGroupName(motion_data.getGroupName());
          trajectory_data.setDuration(durations[k]);
          trajectory_data.trajectory_error_code_.val = errors[k];

          motion_data.addTrajectoryId(trajectory_data.getId());

          if(hasTrajectory(motion_data.getName(),
                            trajectory_data.getName())) {
            ROS_WARN_STREAM("Motion plan request " << motion_data.getName() << " really shouldn't already have a trajectory " << trajectory_data.getName());
          }

          trajectory_map_[motion_data.getName()][trajectory_data.getName()] = trajectory_data;
          //playTrajectory(motion_data,trajectory_map_[motion_data.getName()][trajectory_data.getName()]);
        }
      }
    }
    sendPlanningScene(scene);
    // if(loadRequests) {
    //   for(map<string, MotionPlanRequestData>::iterator it = motion_plan_map_.begin(); it != motion_plan_map_.end(); it ++)
    //   {
    //     if(it->second->hasPathConstraints()) {
    //       solveIKForEndEffectorPose(it->second,
    //                                 GoalPosition,
                                    
    //                                 }
    //     }
    //   }
  }

  interactive_marker_server_->applyChanges();
  unlockScene();
}

void PlanningSceneEditor::getTrajectoryMarkers(visualization_msgs::MarkerArray& arr)
{
  // For each trajectory. 
  for(map<string, map<string, TrajectoryData> >::iterator it = trajectory_map_.begin(); it != trajectory_map_.end(); it++)
  {
    for(map<string, TrajectoryData>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++) {
      // If a trajectory is playing, then show the closest matching pose in the trajectory.
      if(it2->second.isPlaying())
      {
        if(it2->second.getTrajectoryRenderType() == Kinematic)
        {
          // Assume that timestamps are invalid, and render based on index
          it2->second.advanceThroughTrajectory(2);
        }
        else
        {
          // assume timestamps are fine, and render based on time.
          it2->second.advanceToNextClosestPoint(ros::Time::now());
        }

        if( it->first == selected_motion_plan_name_ &&
            it2->first == selected_trajectory_name_ )
        {
          selectedTrajectoryCurrentPointChanged( it2->second.getCurrentPoint() );
        }
      }

      if(it2->second.getCurrentState() == NULL)
      {
        continue;
      }

      it2->second.updateCurrentState();
      
      // When the color of a trajectory has changed, we have to wait for
      // a few milliseconds before the change is registered in rviz.
      if(it2->second.shouldRefreshColors())
      {
        it2->second.refresh_timer_ += marker_dt_;
        
        if(it2->second.refresh_timer_.toSec() > MARKER_REFRESH_TIME + 0.05)
        {
          it2->second.setHasRefreshedColors(true);
          it2->second.refresh_timer_ = ros::Duration(0.0);
        }
      }
      else
      {
        if(it2->second.isVisible())
        {
          ROS_DEBUG_STREAM("Should be showing trajectory for " <<
                           it->first << " " << it2->first
                           << it2->second.getGroupName() << 
                           " " << (cm_->getKinematicModel()->getModelGroup(it2->second.getGroupName()) == NULL));
          const vector<const KinematicModel::LinkModel*>& updated_links =
            cm_->getKinematicModel()->getModelGroup(it2->second.getGroupName())->getUpdatedLinkModels();

          vector<string> lnames;
          lnames.resize(updated_links.size());
          for(unsigned int i = 0; i < updated_links.size(); i++)
          {
            lnames[i] = updated_links[i]->getName();
          }

          // Links in group
          switch(it2->second.getRenderType())
          {
          case VisualMesh:
            cm_->getRobotMarkersGivenState(*(it2->second.getCurrentState()), arr, it2->second.getColor(),
                                           getMotionPlanRequestNameFromId(it2->second.getMotionPlanRequestId())+" "+it2->first + "_trajectory", 
                                           ros::Duration(MARKER_REFRESH_TIME), 
                                           &lnames, 1.0, false);
            // Bodies held by robot
            cm_->getAttachedCollisionObjectMarkers(*(it2->second.getCurrentState()), arr, 
                                                   getMotionPlanRequestNameFromId(it2->second.getMotionPlanRequestId())+" "+it2->first + "_trajectory", 
                                                   it2->second.getColor(), ros::Duration(MARKER_REFRESH_TIME), false, &lnames);

            break;
          case CollisionMesh:
            cm_->getRobotMarkersGivenState(*(it2->second.getCurrentState()), arr, it2->second.getColor(),
                                           getMotionPlanRequestNameFromId(it2->second.getMotionPlanRequestId())+" "+it2->first + "_trajectory", 
                                           ros::Duration(MARKER_REFRESH_TIME), 
                                           &lnames, 1.0, true);
            cm_->getAttachedCollisionObjectMarkers(*(it2->second.getCurrentState()), arr, 
                                                   getMotionPlanRequestNameFromId(it2->second.getMotionPlanRequestId())+" "+it2->first + "_trajectory", 
                                                   it2->second.getColor(), ros::Duration(MARKER_REFRESH_TIME), false, &lnames);
            break;
          case PaddingMesh:
            cm_->getRobotPaddedMarkersGivenState((const KinematicState&)*(it2->second.getCurrentState()),
                                                 arr,
                                                 it2->second.getColor(),
                                                 getMotionPlanRequestNameFromId(it2->second.getMotionPlanRequestId())+" "+it2->first + "_trajectory", 
                                                 ros::Duration(MARKER_REFRESH_TIME)*2.0,
                                                 (const vector<string>*)&lnames);
            cm_->getAttachedCollisionObjectMarkers(*(it2->second.getCurrentState()), arr,
                                                   getMotionPlanRequestNameFromId(it2->second.getMotionPlanRequestId())+" "+it2->first + "_trajectory", 
                                                   it2->second.getColor(), ros::Duration(MARKER_REFRESH_TIME)*2.0, true, &lnames);
            break;
          }

        }
      }


      //////
      /// Get collision markers associated with trajectory.
      /////
      if(it2->second.areCollisionsVisible() && it2->second.isVisible())
      {

        if(motion_plan_map_.find(getMotionPlanRequestNameFromId(it2->second.getMotionPlanRequestId())) == motion_plan_map_.end()) {
          ROS_INFO_STREAM("Making empty mprs in trajectory");
          continue;
        }

        // Update markers
        if(it2->second.hasStateChanged())
        {
          if(params_.proximity_space_validity_name_ == "none")
          {
            it2->second.updateCollisionMarkers(cm_, motion_plan_map_[getMotionPlanRequestNameFromId(it2->second.getMotionPlanRequestId())],
                                               NULL);
          
          } else {
            it2->second.updateCollisionMarkers(cm_, motion_plan_map_[getMotionPlanRequestNameFromId(it2->second.getMotionPlanRequestId())],
                                               &distance_state_validity_service_client_);
          
          }
          it2->second.setStateChanged(false);
        }

        // Then add them to the global array
        for(size_t i = 0; i < it2->second.getCollisionMarkers().markers.size(); i++)
        {
          collision_markers_.markers.push_back(it2->second.getCollisionMarkers().markers[i]);
        }
      }
    }
  }
}

void PlanningSceneEditor::getMotionPlanningMarkers(visualization_msgs::MarkerArray& arr)
{
  vector<string> removals;

  // For each motion plan request ...
  for(map<string, MotionPlanRequestData>::iterator it = motion_plan_map_.begin(); it != motion_plan_map_.end(); it++)
  {
    if(it->second.getName() == "") {
      ROS_WARN("Someone's making empty stuff");
    }
    MotionPlanRequestData& data = it->second;

    // TODO: Find out why this happens.
    if(motion_plan_map_.find(it->first) == motion_plan_map_.end() || data.getName() == "")
    {
      ROS_WARN("Attempting to publish non-existant motion plan request %s Erasing this request!", it->first.c_str());
      removals.push_back(it->first);
      continue;
    }

    // TODO: Find out why this happens.
    if(data.getStartState() == NULL || data.getGoalState() == NULL)
    {
      return;
    }

    // When a motion plan request has its colors changed,
    // we must wait a few milliseconds before rviz registers the change.
    if(data.shouldRefreshColors())
    {
      data.refresh_timer_ += marker_dt_;

      if(data.refresh_timer_.toSec() > MARKER_REFRESH_TIME + 0.05)
      {
        data.setHasRefreshedColors(true);
        data.refresh_timer_ = ros::Duration(0.0);
      }
    }
    else
    {
      std_msgs::ColorRGBA fail_color;
      fail_color.a = 0.9;
      fail_color.r = 1.0;
      fail_color.g = 0.0;
      fail_color.b = 0.0;

      /////
      /// Get markers for the start
      /////
      if(data.isStartVisible())
      {
        const vector<const KinematicModel::LinkModel*>& updated_links =
            cm_->getKinematicModel()->getModelGroup(data.getMotionPlanRequest().group_name)->getUpdatedLinkModels();

        vector<string> lnames;
        lnames.resize(updated_links.size());
        for(unsigned int i = 0; i < updated_links.size(); i++)
        {
          lnames[i] = updated_links[i]->getName();
        }


        // If we have a good ik solution, publish with the normal color
        // else use bright red.
        std_msgs::ColorRGBA col;
        if(data.hasGoodIKSolution(StartPosition))
        {
          col = data.getStartColor();
        } else {
          col = fail_color;
        }
        
        switch(data.getRenderType())
        {
        case VisualMesh:
          cm_->getRobotMarkersGivenState(*(data.getStartState()), arr, col,
                                         it->first + "_start", ros::Duration(MARKER_REFRESH_TIME), 
                                         &lnames, 1.0, false);
          // Bodies held by robot
          cm_->getAttachedCollisionObjectMarkers(*(data.getStartState()), arr, it->first + "_start",
                                                 col, ros::Duration(MARKER_REFRESH_TIME), false, &lnames);
          
          break;
        case CollisionMesh:
          cm_->getRobotMarkersGivenState(*(data.getStartState()), arr, col,
                                         it->first + "_start", ros::Duration(MARKER_REFRESH_TIME), 
                                         &lnames, 1.0, true);
          cm_->getAttachedCollisionObjectMarkers(*(data.getStartState()), arr, it->first + "_start",
                                                 col, ros::Duration(MARKER_REFRESH_TIME), false, &lnames);
          break;
        case PaddingMesh:
          cm_->getRobotPaddedMarkersGivenState(*(data.getStartState()),
                                               arr,
                                               col,
                                               it->first + "_start",
                                               ros::Duration(MARKER_REFRESH_TIME),
                                               (const vector<string>*)&lnames);
          cm_->getAttachedCollisionObjectMarkers(*(data.getStartState()), arr, it->first + "_start",
                                                 col, ros::Duration(MARKER_REFRESH_TIME), true, &lnames);
          break;
        }
      }

      /////
      /// Get markers for the end.
      /////
      if(data.isEndVisible())
      {
        const vector<const KinematicModel::LinkModel*>& updated_links =
            cm_->getKinematicModel()->getModelGroup(data.getMotionPlanRequest().group_name)->getUpdatedLinkModels();

        vector<string> lnames;
        lnames.resize(updated_links.size());
        for(unsigned int i = 0; i < updated_links.size(); i++)
        {
          lnames[i] = updated_links[i]->getName();
        }

        std_msgs::ColorRGBA col;
        if(data.hasGoodIKSolution(GoalPosition))
        {
          col = data.getGoalColor();
        } else {
          col = fail_color;
        }
        
        switch(data.getRenderType())
        {
        case VisualMesh:
          cm_->getRobotMarkersGivenState(*(data.getGoalState()), arr, col,
                                         it->first + "_Goal", ros::Duration(MARKER_REFRESH_TIME),
                                         &lnames, 1.0, false);

          // Bodies held by robot
          cm_->getAttachedCollisionObjectMarkers(*(data.getGoalState()), arr, it->first + "_Goal",
                                                 col, ros::Duration(MARKER_REFRESH_TIME), false, &lnames);
          
          break;
        case CollisionMesh:
          cm_->getRobotMarkersGivenState(*(data.getGoalState()), arr, col,
                                         it->first + "_Goal", ros::Duration(MARKER_REFRESH_TIME),
                                         &lnames, 1.0, true);
          cm_->getAttachedCollisionObjectMarkers(*(data.getGoalState()), arr, it->first + "_Goal",
                                                 col, ros::Duration(MARKER_REFRESH_TIME), false, &lnames);
          break;
        case PaddingMesh:
          cm_->getRobotPaddedMarkersGivenState(*(data.getGoalState()),
                                               arr,
                                               col,
                                               it->first + "_Goal",
                                               ros::Duration(MARKER_REFRESH_TIME),
                                               (const vector<string>*)&lnames);
          cm_->getAttachedCollisionObjectMarkers(*(data.getGoalState()), arr, it->first + "_Goal",
                                                 col, ros::Duration(MARKER_REFRESH_TIME), true, &lnames);
          break;
        }
      }
    }

    //////
    /// Get collision markers for the start and end state.
    /////
    if(it->second.areCollisionsVisible() && (it->second.isStartVisible() || it->second.isEndVisible()))
    {
      // Update collision markers
      if(it->second.hasStateChanged())
      {
        if(params_.proximity_space_validity_name_ == "none")
        {
          it->second.updateCollisionMarkers(cm_, NULL);
        } else {
          it->second.updateCollisionMarkers(cm_, &distance_state_validity_service_client_);
        }
        it->second.setStateChanged(false);
      }

      // Add them to the global array.
      for(size_t i = 0; i < it->second.getCollisionMarkers().markers.size(); i++)
      {
        collision_markers_.markers.push_back(it->second.getCollisionMarkers().markers[i]);
      }
    }

  }

  /////
  /// TODO: Figure out why motion plans are occasionally NULL
  ////
  for(size_t i = 0; i < removals.size(); i++)
  {
    motion_plan_map_.erase(removals[i]);
  }
}

void PlanningSceneEditor::createMotionPlanRequest(const planning_models::KinematicState& start_state,
                                                  const planning_models::KinematicState& end_state, 
                                                  const std::string& group_name,
                                                  const std::string& end_effector_name, 
                                                  const unsigned int& planning_scene_id, 
                                                  const bool from_robot_state,
                                                  unsigned int& motion_plan_id_out)

{
  MotionPlanRequest motion_plan_request;
  motion_plan_request.group_name = group_name;
  motion_plan_request.num_planning_attempts = 1;
  motion_plan_request.allowed_planning_time = ros::Duration(1);
  const KinematicState::JointStateGroup* jsg = end_state.getJointStateGroup(group_name);
  motion_plan_request.goal_constraints.joint_constraints.resize(jsg->getJointNames().size());

  // Must convert kinematic state to robot state message.
  vector<double> joint_values;
  jsg->getKinematicStateValues(joint_values);
  for(unsigned int i = 0; i < jsg->getJointNames().size(); i++)
  {
    motion_plan_request.goal_constraints.joint_constraints[i].joint_name = jsg->getJointNames()[i];
    motion_plan_request.goal_constraints.joint_constraints[i].position = joint_values[i];
    motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.001;
    motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.001;
  }

  // Create start state from kinematic state passed in if robot data is being used
  if(!from_robot_state)
  {
    convertKinematicStateToRobotState(start_state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                      motion_plan_request.start_state);
  }
  // Otherwise, use the current robot state.
  else
  {
    convertKinematicStateToRobotState(*robot_state_, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                      motion_plan_request.start_state);
  }

  if(planning_scene_map_.find(getPlanningSceneNameFromId(planning_scene_id)) == planning_scene_map_.end()) {
    ROS_WARN_STREAM("Creating new planning scene for motion plan request - bad!!");
  }

  PlanningSceneData& planningSceneData = planning_scene_map_[getPlanningSceneNameFromId(planning_scene_id)];
  
  // Turn the motion plan request message into a MotionPlanData
  unsigned int id = planningSceneData.getNextMotionPlanRequestId();
  motion_plan_request.group_name = group_name;
  MotionPlanRequestData data(id, "Planner", motion_plan_request, robot_state_, end_effector_name);
  data.setGoalEditable(true);
  if(from_robot_state)
  {
    data.setStartEditable(false);
  }

  // Book keeping for kinematic state storage
  StateRegistry start;
  start.state = data.getStartState();
  start.source = "Motion Plan Request Data Start create request";
  StateRegistry end;
  end.state = data.getGoalState();
  end.source = "Motion Plan Request Data End from create request";
  states_.push_back(start);
  states_.push_back(end);

  motion_plan_map_[getMotionPlanRequestNameFromId(id)] = data;
  data.setPlanningSceneId(planning_scene_id);

  // Add request to the planning scene
  planningSceneData.addMotionPlanRequestId(id);

  motion_plan_id_out = data.getId();
  createIkControllersFromMotionPlanRequest(data, false);
  sendPlanningScene(planningSceneData);
}

bool PlanningSceneEditor::planToKinematicState(const KinematicState& state, const string& group_name, const string& end_effector_name,
                                               unsigned int& trajectory_id_out, unsigned int& planning_scene_id)
{
  unsigned int motion_plan_id;
  createMotionPlanRequest(*robot_state_, state, group_name, end_effector_name, planning_scene_id,
                          false, motion_plan_id);
  MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(motion_plan_id)];
  return planToRequest(data, trajectory_id_out);
}

bool PlanningSceneEditor::planToRequest(const std::string& request_id, unsigned int& trajectory_id_out)
{
  return planToRequest(motion_plan_map_[request_id], trajectory_id_out);
}

bool PlanningSceneEditor::planToRequest(MotionPlanRequestData& data, unsigned int& trajectory_id_out)
{
  GetMotionPlan::Request plan_req;
  ros::ServiceClient* planner;
  std::string source;
  if(active_planner_index_ == 3) {
    source = "Interpolator";
    arm_navigation_msgs::MotionPlanRequest req;
    req.group_name = data.getGroupName();
    req.num_planning_attempts = 1;
    req.allowed_planning_time = ros::Duration(10.0);
    req.start_state = data.getMotionPlanRequest().start_state;
    req.start_state.multi_dof_joint_state.child_frame_ids[0] = data.getEndEffectorLink();
    req.start_state.multi_dof_joint_state.frame_ids[0] = cm_->getWorldFrameId();
    tf::poseTFToMsg(data.getStartState()->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform(), 
                    req.start_state.multi_dof_joint_state.poses[0]);
    geometry_msgs::PoseStamped end_effector_wrist_pose;
    tf::poseTFToMsg(data.getGoalState()->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform(), 
                    end_effector_wrist_pose.pose);
    end_effector_wrist_pose.header.frame_id = cm_->getWorldFrameId();
    req.goal_constraints.position_constraints.resize(1);
    req.goal_constraints.orientation_constraints.resize(1);
    
    arm_navigation_msgs::poseStampedToPositionOrientationConstraints(end_effector_wrist_pose,
                                                                     data.getEndEffectorLink(),
                                                                     req.goal_constraints.position_constraints[0],
                                                                     req.goal_constraints.orientation_constraints[0]);
    
    plan_req.motion_plan_request = req;
    planner = (*interpolated_ik_services_)[data.getEndEffectorLink()];
  } else {
    source = "Planner";
    if( active_planner_index_ == 2 )
    {
      planner = &planning_2_service_client_;
    }
    else
    {
      planner = &planning_1_service_client_;
    }

    if(data.hasPathConstraints()) {
      data.setGoalAndPathPositionOrientationConstraints(plan_req.motion_plan_request, GoalPosition);
      plan_req.motion_plan_request.group_name += "_cartesian";
    } else {
      plan_req.motion_plan_request = data.getMotionPlanRequest();
      plan_req.motion_plan_request.goal_constraints.position_constraints.clear();
      plan_req.motion_plan_request.goal_constraints.orientation_constraints.clear();
      plan_req.motion_plan_request.path_constraints.position_constraints.clear();
      plan_req.motion_plan_request.path_constraints.orientation_constraints.clear();
    }
    plan_req.motion_plan_request.allowed_planning_time = ros::Duration(10.0);
  }
  GetMotionPlan::Response plan_res;

  if(!planner->call(plan_req, plan_res))
  {
    ROS_INFO("Something wrong with planner client");
    planCallback(plan_res.error_code);
    return false;
  }

  unsigned int id = data.getNextTrajectoryId();

  TrajectoryData trajectory_data;
  trajectory_data.setTrajectory(plan_res.trajectory.joint_trajectory);
  trajectory_data.setGroupName(data.getMotionPlanRequest().group_name);
  trajectory_data.setMotionPlanRequestId(data.getId());
  trajectory_id_out = id;
  trajectory_data.setPlanningSceneId(data.getPlanningSceneId());
  trajectory_data.setId(trajectory_id_out);
  trajectory_data.setSource(source);
  trajectory_data.setDuration(plan_res.planning_time);
  trajectory_data.setVisible(true);
  trajectory_data.setTrajectoryRenderType(Kinematic);
  trajectory_data.play();

  bool success = (plan_res.error_code.val == plan_res.error_code.SUCCESS);
  trajectory_data.trajectory_error_code_.val = plan_res.error_code.val;
  lockScene();
  data.addTrajectoryId(id);
  trajectory_map_[data.getName()][trajectory_data.getName()] = trajectory_data;
  unlockScene();

  if(success) {
    selected_trajectory_name_ = trajectory_data.getName();
  } else {
    ROS_INFO_STREAM("Bad planning error code " << plan_res.error_code.val);
  }
  planCallback(trajectory_data.trajectory_error_code_);
  return success;
}

// void PlanningSceneEditor::determineOrientationConstraintsGivenState(const MotionPlanRequestData& mpr,
//                                                                     const KinematicState& state,
//                                                                     OrientationConstraint& goal_constraint,
//                                                                     OrientationConstraint& path_constraint)
// {
//   tf::Transform cur = state.getLinkState(mpr.getEndEffectorLink())->getGlobalLinkTransform();
//   //tfScalar roll, pitch, yaw;
//   //cur.getBasis().getRPY(roll,pitch,yaw);
//   goal_constraint.header.frame_id = cm_->getWorldFrameId();
//   goal_constraint.header.stamp = ros::Time(ros::WallTime::now().toSec());
//   goal_constraint.link_name = mpr.getEndEffectorLink();
//   tf::quaternionTFToMsg(cur.getRotation(), goal_constraint.orientation);
//   goal_constraint.absolute_roll_tolerance = 0.04;
//   goal_constraint.absolute_pitch_tolerance = 0.04;
//   goal_constraint.absolute_yaw_tolerance = 0.04;

//   path_constraint.header.frame_id = cm_->getWorldFrameId();
//   path_constraint.header.stamp = ros::Time(ros::WallTime::now().toSec());
//   path_constraint.link_name = mpr.getEndEffectorLink();
//   tf::quaternionTFToMsg(cur.getRotation(), path_constraint.orientation);
//   path_constraint.type = path_constraint.HEADER_FRAME;
//   if(mpr.getConstrainRoll()) {
//     path_constraint.absolute_roll_tolerance = mpr.getRollTolerance();
//   } else {
//     path_constraint.absolute_roll_tolerance = M_PI;
//   }
//   if(mpr.getConstrainPitch()) {
//     path_constraint.absolute_pitch_tolerance = mpr.getPitchTolerance();
//   } else {
//     path_constraint.absolute_pitch_tolerance = M_PI;
//   }
//   if(mpr.getConstrainYaw()) {
//     path_constraint.absolute_yaw_tolerance = mpr.getYawTolerance();
//   } else {
//     path_constraint.absolute_yaw_tolerance = M_PI;
//   }
// }

void PlanningSceneEditor::printTrajectoryPoint(const vector<string>& joint_names, const vector<double>& joint_values)
{
  for(unsigned int i = 0; i < joint_names.size(); i++)
  {
    ROS_INFO_STREAM("Joint name " << joint_names[i] << " value " << joint_values[i]);
  }
}

bool PlanningSceneEditor::filterTrajectory(MotionPlanRequestData& requestData, TrajectoryData& trajectory,
                                           unsigned int& filter_id)
{
  FilterJointTrajectoryWithConstraints::Request filter_req;
  FilterJointTrajectoryWithConstraints::Response filter_res;

  // Filter request has to have robot state message filled
  convertKinematicStateToRobotState(*robot_state_, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                    filter_req.start_state);

  filter_req.trajectory = trajectory.getTrajectory();
  filter_req.group_name = trajectory.getGroupName();
  filter_req.goal_constraints = requestData.getMotionPlanRequest().goal_constraints;
  filter_req.path_constraints = requestData.getMotionPlanRequest().path_constraints;
  filter_req.allowed_time = ros::Duration(2.0);

  // Select the filter
  ros::ServiceClient* trajectory_filter_service_client;
  if( use_primary_filter_ )
  {
    trajectory_filter_service_client = &trajectory_filter_1_service_client_;
  }
  else
  {
    trajectory_filter_service_client = &trajectory_filter_2_service_client_;
  }

  // Time the filtering
  ros::Time startTime = ros::Time(ros::WallTime::now().toSec());
  if(!trajectory_filter_service_client->call(filter_req, filter_res))
  {
    ROS_INFO("Problem with trajectory filter");
    filterCallback(filter_res.error_code);
    return false;
  }

  unsigned int id = requestData.getNextTrajectoryId();

  // Convert returned joint trajectory to TrajectoryData
  TrajectoryData data(id, "Trajectory Filterer", trajectory.getGroupName(),
                      filter_res.trajectory);
  data.setPlanningSceneId(requestData.getPlanningSceneId());
  data.setMotionPlanRequestId(requestData.getId());
  data.setDuration(ros::Time(ros::WallTime::now().toSec()) - startTime);
  data.setTrajectoryRenderType(Temporal);
  requestData.addTrajectoryId(id);

  data.trajectory_error_code_.val = filter_res.error_code.val;
  trajectory_map_[requestData.getName()][data.getName()] = data;
  filter_id = data.getId();
  data.setVisible(true);
  data.play();
  selected_trajectory_name_ = data.getName();

  bool success = (filter_res.error_code.val == filter_res.error_code.SUCCESS);
  if(!success) {
    ROS_INFO_STREAM("Bad trajectory_filter error code " << filter_res.error_code.val);
  } else {
    playTrajectory(requestData, trajectory_map_[requestData.getName()][data.getName()]);
  }
  filterCallback(filter_res.error_code);
  return success;
}

void PlanningSceneEditor::updateJointStates()
{

  // If using robot data, the joint states are handled by whatever is
  // running on the robot (presumably)
  if(params_.use_robot_data_)
  {
    return;
  }

  sensor_msgs::JointState msg;
  msg.header.frame_id = cm_->getWorldFrameId();
  msg.header.stamp = ros::Time(ros::WallTime::now().toSec());

  vector<KinematicState::JointState*> jointStates = getRobotState()->getJointStateVector();

  map<string, double> stateMap;
  getRobotState()->getKinematicStateValues(stateMap);
  getRobotState()->setKinematicState(stateMap);

  // Send each joint state out as part of a message to the robot state publisher.
  for(size_t i = 0; i < jointStates.size(); i++)
  {
    KinematicState::JointState* state = jointStates[i];
    msg.name.push_back(state->getName());

    // Assume that joints only have one value.
    if(state->getJointStateValues().size() > 0)
    {
      msg.position.push_back(state->getJointStateValues()[0]);
    }
    else
    {
      msg.position.push_back(0.0f);
    }
  }
  joint_state_publisher_.publish(msg);

}

void PlanningSceneEditor::sendMarkers()
{
  marker_dt_ = (ros::Time::now() - last_marker_start_time_);
  last_marker_start_time_ = ros::Time::now();
  lockScene();
  sendTransformsAndClock();
  visualization_msgs::MarkerArray arr;

  getTrajectoryMarkers(arr);
  getMotionPlanningMarkers(arr);

  vis_marker_array_publisher_.publish(arr);
  vis_marker_array_publisher_.publish(collision_markers_);
  collision_markers_.markers.clear();
  unlockScene();
}

bool PlanningSceneEditor::getPlanningSceneOutcomes(const unsigned int id,
                                                   vector<string>& pipeline_stages,
                                                   vector<ArmNavigationErrorCodes>& error_codes,
                                                   map<std::string, ArmNavigationErrorCodes>& error_map)
{
  if(!move_arm_warehouse_logger_reader_->getAssociatedOutcomes("", id, pipeline_stages, error_codes))
  {
    ROS_DEBUG_STREAM("No outcome associated with planning scene");
    return false;
  }

  // Fill error map for convenience
  for(size_t i = 0; i < error_codes.size(); i++)
  {
    error_map[pipeline_stages[i]] = error_codes[i];
  }

  return true;
}

std::string PlanningSceneEditor::createNewPlanningScene()
{
  lock_scene_.lock();

  if(robot_state_ == NULL)
  {
    robot_state_ = new KinematicState(cm_->getKinematicModel());
  } else {
    if(robot_state_joint_values_.empty()) {
      robot_state_->setKinematicStateToDefault();
    } else {
      robot_state_->setKinematicState(robot_state_joint_values_);
    }
  }
  
  PlanningSceneData data;
  data.setId(generateNewPlanningSceneId());
  data.setTimeStamp(ros::Time(ros::WallTime::now().toSec()));

  convertKinematicStateToRobotState(*robot_state_, data.getTimeStamp(), cm_->getWorldFrameId(),
                                    data.getPlanningScene().robot_state);
  //end_effector_state_ = planning_state_;

  std::vector<string> collisionObjects;

  for(map<string, SelectableObject>::iterator it = selectable_objects_->begin(); it != selectable_objects_->end(); it++)
  {
    collisionObjects.push_back(it->first);
  }

  //this also does attached collision objects
  for(size_t i = 0; i < collisionObjects.size(); i++)
  {
    deleteCollisionObject(collisionObjects[i]);
  }

  selectable_objects_->clear();

  char hostname[256];
  gethostname(hostname, 256);
  data.setHostName(std::string(hostname));

  planning_scene_map_[data.getName()] = data;
  lock_scene_.unlock();

  updateJointStates();

  return data.getName();
}

void PlanningSceneEditor::loadAllWarehouseData()
{
  max_collision_object_id_ = 0;

  motion_plan_map_.clear();
  trajectory_map_.clear();
  planning_scene_map_.clear();
  vector<ros::Time> planning_scene_times;
  vector<unsigned int> planning_scene_ids;
  getAllPlanningSceneTimes(planning_scene_times,
                           planning_scene_ids);
  
  ROS_INFO_STREAM("Starting load");

  // For each planning scene
  for(size_t i = 0; i < planning_scene_times.size(); i++)
  {
    ros::Time& time = planning_scene_times[i];
    // Load it
    loadPlanningScene(time, planning_scene_ids[i]);

    ROS_DEBUG_STREAM("Got planning scene " << planning_scene_ids[i] << " from warehouse.");
    PlanningSceneData& data = planning_scene_map_[getPlanningSceneNameFromId(planning_scene_ids[i])];
    data.getPipelineStages().clear();
    data.getErrorCodes().clear();
    getPlanningSceneOutcomes(planning_scene_ids[i], data.getPipelineStages(), data.getErrorCodes(), error_map_);
    onPlanningSceneLoaded((int)i, (int)planning_scene_times.size());
  }

  error_map_.clear();
}

void PlanningSceneEditor::savePlanningScene(PlanningSceneData& data, bool copy)
{
  PlanningScene* actual_planning_scene;
  unsigned int id_to_push;

  std::string name_to_push = "";

  warehouse_data_loaded_once_ = false;
  
  //overwriting timestamp
  data.setTimeStamp(ros::Time(ros::WallTime::now().toSec()));

  // Have to do this in case robot state was corrupted by sim time.
  if(!copy) {
    bool has;
    has = move_arm_warehouse_logger_reader_->hasPlanningScene(data.getHostName(),
                                                              data.getId());
    //ROS_INFO_STREAM("Has is " << has);

    if(has) {
      move_arm_warehouse_logger_reader_->removePlanningSceneAndAssociatedDataFromWarehouse(data.getHostName(),
                                                                                           data.getId());
    }
    id_to_push = data.getId();

    actual_planning_scene = &(data.getPlanningScene());

    ROS_INFO("Saving Planning Scene %s", data.getName().c_str());
  } else {
    //force reload
    PlanningSceneData ndata = data;
    ndata.setId(generateNewPlanningSceneId());
    id_to_push = ndata.getId();
    ROS_INFO("Copying Planning Scene %s to %s", data.getName().c_str(), ndata.getName().c_str());
    planning_scene_map_[ndata.getName()] = ndata;
    name_to_push = ndata.getName();
    actual_planning_scene = &planning_scene_map_[ndata.getName()].getPlanningScene();
  }

  move_arm_warehouse_logger_reader_->pushPlanningSceneToWarehouse(*actual_planning_scene, 
                                                                  id_to_push);
  
  for(std::set<unsigned int>::iterator it = data.getRequests().begin(); it != data.getRequests().end(); it++) {
    MotionPlanRequestData& req = motion_plan_map_[getMotionPlanRequestNameFromId(*it)];
    MotionPlanRequest mpr;
    if(req.hasPathConstraints()) {
      req.setGoalAndPathPositionOrientationConstraints(mpr, GoalPosition);
    } else {
      mpr = req.getMotionPlanRequest();
    }
    move_arm_warehouse_logger_reader_->pushMotionPlanRequestToWarehouse(id_to_push, 
                                                                        req.getId(),
                                                                        req.getSource(),
                                                                        mpr);
    ROS_DEBUG_STREAM("Saving Request " << req.getId());
    for(std::set<unsigned int>::iterator it2 = req.getTrajectories().begin(); it2 != req.getTrajectories().end(); it2++) {
      TrajectoryData& traj = trajectory_map_[req.getName()][getTrajectoryNameFromId(*it2)];
      move_arm_warehouse_logger_reader_->pushJointTrajectoryToWarehouse(id_to_push,
                                                                        traj.getSource(),
                                                                        traj.getDuration(), 
                                                                        traj.getTrajectory(),
                                                                        traj.getTrajectoryError(),
                                                                        traj.getId(),
                                                                        traj.getMotionPlanRequestId(), 
                                                                        traj.trajectory_error_code_);
      move_arm_warehouse_logger_reader_->pushOutcomeToWarehouse(id_to_push,
                                                                traj.getSource(),
                                                                traj.trajectory_error_code_);
      ROS_DEBUG_STREAM("Saving Trajectory " << traj.getId());
    }
  }
  if(!name_to_push.empty()) {
    setCurrentPlanningScene(name_to_push);
  }
}

bool PlanningSceneEditor::getAllPlanningSceneTimes(vector<ros::Time>& planning_scene_times,
                                                   vector<unsigned int>& planning_scene_ids)
{
  move_arm_warehouse_logger_reader_->getAvailablePlanningSceneList("", 
                                                                   planning_scene_ids,
                                                                   last_creation_time_query_);
  planning_scene_times = last_creation_time_query_;
  return true;
}

bool PlanningSceneEditor::loadPlanningScene(const ros::Time& time, 
                                            const unsigned int id)
{
  PlanningSceneData data;
  data.setTimeStamp(time);
  data.setId(id);
  std::string host;
  if(!move_arm_warehouse_logger_reader_->getPlanningScene("", id, data.getPlanningScene(), host))
  {
    return false;
  }

  data.setHostName(host);

  std::pair<string, PlanningSceneData> p(data.getName(), data);
  planning_scene_map_.insert(p);
  return true;
}

bool PlanningSceneEditor::getAllAssociatedMotionPlanRequests(const unsigned int id,
                                                             vector<unsigned int>& ids,
                                                             vector<string>& stages,
                                                             vector<MotionPlanRequest>& requests)
{
  move_arm_warehouse_logger_reader_->getAssociatedMotionPlanRequests("", id, ids, stages, requests);
  return true;
}

void PlanningSceneEditor::deleteKinematicStates()
{
  lockScene();
  std::vector<KinematicState*> removals;
  for(map<string, map<string, TrajectoryData> >::iterator it = trajectory_map_.begin(); it != trajectory_map_.end(); it++)
  {
    for(map<string, TrajectoryData>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++) {
      removals.push_back(it2->second.getCurrentState());
      it2->second.reset();
    }
  }

  for(map<string, MotionPlanRequestData>::iterator it = motion_plan_map_.begin(); it != motion_plan_map_.end(); it++)
  {
    removals.push_back(it->second.getStartState());
    removals.push_back(it->second.getGoalState());
    it->second.reset();
  }

  for(size_t i = 0; i < states_.size(); i++)
  {
    if(states_[i].state != NULL)
    {
      bool shouldBreak = false;
      for(size_t j = 0; j < removals.size(); j++)
      {
        if(states_[i].state == removals[j])
        {
          shouldBreak = true;
          break;
        }
      }

      if(shouldBreak)
      {
        continue;
      }
      ROS_INFO("Missed a state from %s!", states_[i].source.c_str());
      delete states_[i].state;
      states_[i].state = NULL;
    }
  }
  states_.clear();
  unlockScene();
}

bool PlanningSceneEditor::sendPlanningScene(PlanningSceneData& data)
{
  lockScene();
  last_collision_set_error_code_.val = 0;

  SetPlanningSceneDiff::Request planning_scene_req;
  SetPlanningSceneDiff::Response planning_scene_res;

  planning_scene_req.planning_scene_diff = data.getPlanningScene();
  if(params_.use_robot_data_) {
    //just will get latest data from the monitor
    arm_navigation_msgs::RobotState emp;
    planning_scene_req.planning_scene_diff.robot_state = emp;
  }

  collision_space::EnvironmentModel::AllowedCollisionMatrix acm; 
  if(planning_scene_req.planning_scene_diff.allowed_collision_matrix.link_names.empty()) {
    acm = cm_->getDefaultAllowedCollisionMatrix();
  } else {
    acm = planning_environment::convertFromACMMsgToACM(planning_scene_req.planning_scene_diff.allowed_collision_matrix);
  }
  planning_scene_req.planning_scene_diff.collision_objects = std::vector<CollisionObject>();
  planning_scene_req.planning_scene_diff.attached_collision_objects = std::vector<AttachedCollisionObject>();
  deleteKinematicStates();

  if(robot_state_ != NULL)
  {
    cm_->revertPlanningScene(robot_state_);
    robot_state_ = NULL;
  }

  vector<string> removals;
  // Handle additions and removals of planning scene objects.
  for(map<string, SelectableObject>::iterator it = (*selectable_objects_).begin(); it
      != (*selectable_objects_).end(); it++)
  {
    string name = it->first;
    arm_navigation_msgs::CollisionObject& object = it->second.collision_object_;

    if(it->second.attach_) {
      if(acm.hasEntry(object.id)) {
        acm.changeEntry(object.id, false);
      } else {
        acm.addEntry(object.id, false);
      }
      //first doing group expansion of touch links
      std::vector<std::string>& touch_links = it->second.attached_collision_object_.touch_links;
      std::vector<std::string> modded_touch_links;
      for(unsigned int i = 0; i < touch_links.size(); i++) {
        if(cm_->getKinematicModel()->getModelGroup(touch_links[i])) {
          std::vector<std::string> links = cm_->getKinematicModel()->getModelGroup(touch_links[i])->getGroupLinkNames();
          modded_touch_links.insert(modded_touch_links.end(), links.begin(), links.end());
        } else {
          modded_touch_links.push_back(touch_links[i]);
        }
      }
      std::string& link_name = it->second.attached_collision_object_.link_name;
      if(find(modded_touch_links.begin(), modded_touch_links.end(), link_name) == modded_touch_links.end()) {
        modded_touch_links.push_back(link_name);
      }
      touch_links = modded_touch_links;
      acm.changeEntry(object.id, touch_links, true);
      it->second.attached_collision_object_.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD; 
      it->second.attach_ = false;
    } else if(it->second.detach_) {
      if(acm.hasEntry(object.id)) {
        acm.changeEntry(object.id, false);
      }
      (*selectable_objects_)[name].attached_collision_object_.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;      
      (*selectable_objects_)[name].collision_object_.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
      (*selectable_objects_)[name].detach_ = false;
    }

    if(it->second.attached_collision_object_.object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD) {
      planning_scene_req.planning_scene_diff.attached_collision_objects.push_back(it->second.attached_collision_object_);
    } else if(object.operation.operation != arm_navigation_msgs::CollisionObjectOperation::REMOVE) {
      if(!acm.hasEntry(object.id)) {
        acm.addEntry(object.id, false);
      }
      planning_scene_req.planning_scene_diff.collision_objects.push_back(object);
    } else {
      if(acm.hasEntry(object.id)) {
        acm.removeEntry(object.id);
      }
      removals.push_back(it->first);
      interactive_marker_server_->erase(it->first);
    }
  }

  planning_environment::convertFromACMToACMMsg(acm, planning_scene_req.planning_scene_diff.allowed_collision_matrix);

  if(!set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res))
  {
    ROS_WARN("Can't get planning scene");
    unlockScene();
    return false;
  }

  // Delete collision poles from the map which were removed.
  for(size_t i = 0; i < removals.size(); i++)
  {
    selectable_objects_->erase(removals[i]);
  }

  data.setPlanningScene(planning_scene_res.planning_scene);

  setRobotState(cm_->setPlanningScene(data.getPlanningScene()));

  if(getRobotState() == NULL)
  {
    ROS_ERROR("Something wrong with planning scene");
    unlockScene();
    return false;
  }

  std_msgs::ColorRGBA add_color;
  add_color.a = 1.0f;
  add_color.r = 0.0f;
  add_color.g = 1.0f;
  add_color.b = 0.0f;

  //check if there are new objects we don't know about
  for(unsigned int i = 0; i < planning_scene_res.planning_scene.collision_objects.size(); i++) {
    arm_navigation_msgs::CollisionObject& coll = planning_scene_res.planning_scene.collision_objects[i];
    if(selectable_objects_->find(coll.id) == selectable_objects_->end()) {
      createSelectableMarkerFromCollisionObject(coll,
                                                coll.id,
                                                coll.id,
                                                add_color,
                                                true);
    }
  }

  for(unsigned int i = 0; i < planning_scene_res.planning_scene.attached_collision_objects.size(); i++) {
    arm_navigation_msgs::CollisionObject& coll = planning_scene_res.planning_scene.attached_collision_objects[i].object;
    if(selectable_objects_->find(coll.id) == selectable_objects_->end()) {
      makeSelectableAttachedObjectFromPlanningScene(planning_scene_res.planning_scene,
                                                    planning_scene_res.planning_scene.attached_collision_objects[i]);
    }
  }

  //Now we may need to update the positions of attached collision objects
  for(std::map<std::string, SelectableObject>::iterator it = selectable_objects_->begin();
      it != selectable_objects_->end(); 
      it++) {
    if(it->second.attached_collision_object_.object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD) {
      planning_models::KinematicState::AttachedBodyState* att_state = 
        robot_state_->getAttachedBodyState(it->second.attached_collision_object_.object.id);
      if(att_state == NULL) {
        ROS_WARN_STREAM("Some problem with " << it->first);
        continue;
      } 
      std_msgs::Header header;
      header.frame_id = cm_->getWorldFrameId();
      header.stamp = ros::Time::now();
      interactive_marker_server_->setPose(it->second.selection_marker_.name,
                                          toGeometryPose(att_state->getGlobalCollisionBodyTransforms()[0]),
                                          header);
      interactive_marker_server_->applyChanges();
      it->second.collision_object_.poses[0] = toGeometryPose(att_state->getGlobalCollisionBodyTransforms()[0]);
    }
  }

  robot_state_->getKinematicStateValues(robot_state_joint_values_);

  for(map<string, MotionPlanRequestData>::iterator it = motion_plan_map_.begin(); it != motion_plan_map_.end(); it++)
  {
    it->second.setStartState(new KinematicState(*robot_state_));
    it->second.setGoalState(new KinematicState(*robot_state_));
    StateRegistry start;
    start.state = it->second.getStartState();
    start.source = "Motion Plan Start After Sending Scene";
    states_.push_back(start);
    StateRegistry end;
    end.state = it->second.getGoalState();
    end.source = "Motion Plan End After Sending Scene";
    states_.push_back(end);
    it->second.updateStartState();
    it->second.updateGoalState();
  }

  //setShowCollisions(false);

  unlockScene();
  return true;
}

void PlanningSceneEditor::initMotionPlanRequestData(const unsigned int& planning_scene_id, 
                                                    const std::vector<unsigned int>& ids,
                                                    const std::vector<std::string>& stages,
                                                    const std::vector<arm_navigation_msgs::MotionPlanRequest>& requests)
{
  lockScene();
  for(size_t i = 0; i < requests.size(); i++)
  {
    const MotionPlanRequest& mpr = requests[i];
    cm_->disableCollisionsForNonUpdatedLinks(mpr.group_name);

    setRobotStateAndComputeTransforms(mpr.start_state, *robot_state_);

    GetMotionPlan::Request plan_req;
    plan_req.motion_plan_request = mpr;
    GetMotionPlan::Response plan_res;

    if(params_.proximity_space_service_name_ != "none") {
      if(!distance_aware_service_client_.call(plan_req, plan_res))
      {
        ROS_INFO("Something wrong with distance client");
      }
    }

    const KinematicModel::GroupConfig& config =
        cm_->getKinematicModel()->getJointModelGroupConfigMap().at(mpr.group_name);
    std::string tip = config.tip_link_;

    MotionPlanRequestData data(ids[i],
                               stages[i], 
                               mpr,
                               robot_state_,
                               tip);
    data.setPlanningSceneId(planning_scene_id);

    StateRegistry start;
    start.state = data.getStartState();
    start.source = "Motion Plan Request Data Start from createRequest";
    StateRegistry end;
    end.state = data.getGoalState();
    end.source = "Motion Plan Request Data End from createRequest";
    states_.push_back(start);
    states_.push_back(end);


    PlanningSceneData& planningSceneData = planning_scene_map_[getPlanningSceneNameFromId(planning_scene_id)];
    planningSceneData.addMotionPlanRequestId(ids[i]);

    std::vector<unsigned int> erased_trajectories;
    if(motion_plan_map_.find(data.getName()) != motion_plan_map_.end())
    {
      ROS_INFO_STREAM("Shouldn't be replacing trajectories here");
      deleteMotionPlanRequest(data.getId(), erased_trajectories);
    }
    motion_plan_map_[getMotionPlanRequestNameFromId(data.getId())] = data;

    createIkControllersFromMotionPlanRequest(data, false);
  }
  unlockScene();
}

// bool PlanningSceneEditor::getMotionPlanRequest(const ros::Time& time, const unsigned int id, const string& stage, MotionPlanRequest& mpr,
//                                                string& id, string& planning_scene_id)
// {
//   if(!move_arm_warehouse_logger_reader_->getAssociatedMotionPlanRequest("", id, time, stage, mpr))
//   {
//     ROS_INFO_STREAM("No request with stage " << stage);
//     return false;
//   }

//   lockScene();
//   cm_->disableCollisionsForNonUpdatedLinks(mpr.group_name);

//   setRobotStateAndComputeTransforms(mpr.start_state, *robot_state_);

//   GetMotionPlan::Request plan_req;
//   plan_req.motion_plan_request = mpr;
//   GetMotionPlan::Response plan_res;

//   if(params_.proximity_space_service_name_ != "none") {
//     if(!distance_aware_service_client_.call(plan_req, plan_res))
//     {
//       ROS_INFO("Something wrong with distance client");
//     }
//   }

//   MotionPlanRequestData data(robot_state_);
//   data.setId(generateNewMotionPlanId());

//   data.setMotionPlanRequest(mpr);
//   data.setPlanningSceneName(planning_scene_id);
//   data.setGroupName(mpr.group_name);
//   data.setSource("Planner");
//   StateRegistry start;
//   start.state = data.getStartState();
//   start.source = "Motion Plan Request Data Start from loadRequest";
//   StateRegistry end;
//   end.state = data.getGoalState();
//   end.source = "Motion Plan Request Data End from line loadRequest";
//   states_.push_back(start);
//   states_.push_back(end);

//   const KinematicModel::GroupConfig& config =
//       cm_->getKinematicModel()->getJointModelGroupConfigMap().at(mpr.group_name);
//   std::string tip = config.tip_link_;
//   data.setEndEffectorLink(tip);

//   PlanningSceneData& planningSceneData = planning_scene_map_[planning_scene_id];
//   planningSceneData.getRequests().push_back(data.getId());

//   motion_plan_map_[data.getId()] = data;
//   id = data.getId();

//   lock_scene_.unlock();

//   createIkControllersFromMotionPlanRequest(data, false);
//   return true;
// }

bool PlanningSceneEditor::getAllAssociatedTrajectorySources(const unsigned int planning_id, 
                                                            const unsigned int mpr_id, 
                                                            vector<unsigned int>& trajectory_ids,
                                                            vector<string>& trajectory_sources)
{
  if(!move_arm_warehouse_logger_reader_->getAssociatedJointTrajectorySources("", planning_id, mpr_id, 
                                                                             trajectory_ids,
                                                                             trajectory_sources))
  {
    return false;
  }
  return true;
}

bool PlanningSceneEditor::getAllAssociatedPausedStates(const unsigned int id, 
                                                       vector<ros::Time>& paused_times)
{
  if(!move_arm_warehouse_logger_reader_->getAssociatedPausedStates("", id, paused_times))
  {
    return false;
  }
  return true;
}

bool PlanningSceneEditor::getPausedState(const unsigned int id, 
                                         const ros::Time& paused_time,
                                         HeadMonitorFeedback& paused_state)
{
  if(!move_arm_warehouse_logger_reader_->getAssociatedPausedState("", id, paused_time, paused_state))
  {
    return false;
  }
  return true;
}

bool PlanningSceneEditor::playTrajectory(MotionPlanRequestData& requestData, TrajectoryData& data)
{
  lock_scene_.lock();
  for(size_t i = 0; i < states_.size(); i++)
  {
    if(states_[i].state == data.getCurrentState())
    {
      states_[i].state = NULL;
    }
  }

  data.reset();

  data.play();
  data.setVisible(true);
  if(data.getTrajectory().points.size() == 0)
  {
    lock_scene_.unlock();
    return false;
  }

  if(data.getCurrentState() == NULL)
  {
    data.setCurrentState(new KinematicState(*robot_state_));
    StateRegistry currentState;
    currentState.state = data.getCurrentState();
    currentState.source = "Trajectory from play trajectory";
    states_.push_back(currentState);
  }

  data.setCurrentPoint(0);
  ArmNavigationErrorCodes oldValue;
  oldValue.val = data.trajectory_error_code_.val;
  ArmNavigationErrorCodes& errorCode = data.trajectory_error_code_;
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = cm_->getCurrentAllowedCollisionMatrix();
  cm_->disableCollisionsForNonUpdatedLinks(data.getGroupName());

  vector<ArmNavigationErrorCodes> trajectory_error_codes;
  arm_navigation_msgs::Constraints path_constraints;
  arm_navigation_msgs::Constraints goal_constraints;
  if(requestData.hasPathConstraints()) {
    path_constraints = requestData.getMotionPlanRequest().path_constraints;
    goal_constraints.position_constraints = requestData.getMotionPlanRequest().goal_constraints.position_constraints;
    goal_constraints.orientation_constraints = requestData.getMotionPlanRequest().goal_constraints.orientation_constraints;
  } else {
    goal_constraints.joint_constraints = requestData.getMotionPlanRequest().goal_constraints.joint_constraints;
  }
  
  cm_->isJointTrajectoryValid(*(data.getCurrentState()), data.getTrajectory(),
                              goal_constraints,
                              path_constraints, errorCode,
                              trajectory_error_codes, false);

  cm_->setAlteredAllowedCollisionMatrix(acm);

  if(errorCode.val != errorCode.SUCCESS)
  {
    if(trajectory_error_codes.size() > 0)
    {
      data.setBadPoint(trajectory_error_codes.size() - 1);
    }
    else
    {
      data.setBadPoint(0);
    }
  }
  else
  {
    data.setBadPoint(-1);
    errorCode.val = oldValue.val;
  }

  data.setCurrentPoint(0);
  lock_scene_.unlock();
  return true;
}

void PlanningSceneEditor::createSelectableMarkerFromCollisionObject(CollisionObject& object, 
                                                                    string name,
                                                                    string description, 
                                                                    std_msgs::ColorRGBA color,
                                                                    bool insert_selection)
{
  SelectableObject selectable;
  selectable.id_ = name;
  selectable.collision_object_ = object;
  selectable.control_marker_.pose = object.poses[0];
  selectable.control_marker_.header.frame_id = "/" + cm_->getWorldFrameId();
  selectable.control_marker_.header.stamp = ros::Time(ros::WallTime::now().toSec());

  selectable.selection_marker_.pose = object.poses[0];
  selectable.selection_marker_.header.frame_id = "/" + cm_->getWorldFrameId();
  selectable.selection_marker_.header.stamp = ros::Time(ros::WallTime::now().toSec());

  selectable.color_ = color;

  InteractiveMarkerControl button;
  button.name = "button";
  button.interaction_mode = InteractiveMarkerControl::BUTTON;
  button.description = "";

  //min scale initialized
  double scale_to_use = .2f;

  for(size_t i = 0; i < object.shapes.size(); i++)
  {
    arm_navigation_msgs::Shape& shape = object.shapes[i];
    Marker mark;
    mark.color = color;
    planning_environment::setMarkerShapeFromShape(shape, mark);

    shapes::Shape* s = planning_environment::constructObject(shape);
    bodies::Body* b = bodies::createBodyFromShape(s);
    
    bodies::BoundingSphere bs;
    b->computeBoundingSphere(bs);

    delete b;
    delete s;

    if(bs.radius * 2.0 > scale_to_use) {
      scale_to_use = bs.radius*2.0;
    }

    //need to make slightly larger
    mark.scale.x = mark.scale.x * 1.01f;
    mark.scale.y = mark.scale.y * 1.01f;
    mark.scale.z = mark.scale.z * 1.01f;
    
    if(mark.type == Marker::LINE_LIST) {
      mark.points.clear();
      mark.type = Marker::TRIANGLE_LIST;
      mark.scale.x = 1.01;
      mark.scale.y = 1.01;
      mark.scale.z = 1.01;
      for(unsigned int i = 0; i < shape.triangles.size(); i += 3) { 
        mark.points.push_back(shape.vertices[shape.triangles[i]]);
        mark.points.push_back(shape.vertices[shape.triangles[i+1]]);
        mark.points.push_back(shape.vertices[shape.triangles[i+2]]);
      }
    }

    button.markers.push_back(mark);
  }
  
  selectable.selection_marker_.controls.push_back(button);

  InteractiveMarkerControl sixDof;
  sixDof.orientation.w = 1;
  sixDof.orientation.x = 1;
  sixDof.orientation.y = 0;
  sixDof.orientation.z = 0;
  sixDof.always_visible = false;
  sixDof.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  selectable.control_marker_.controls.push_back(sixDof);
  sixDof.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  selectable.control_marker_.controls.push_back(sixDof);

  sixDof.orientation.w = 1;
  sixDof.orientation.x = 0;
  sixDof.orientation.y = 1;
  sixDof.orientation.z = 0;
  sixDof.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  selectable.control_marker_.controls.push_back(sixDof);
  sixDof.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  selectable.control_marker_.controls.push_back(sixDof);

  sixDof.orientation.w = 1;
  sixDof.orientation.x = 0;
  sixDof.orientation.y = 0;
  sixDof.orientation.z = 1;
  sixDof.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  selectable.control_marker_.controls.push_back(sixDof);
  sixDof.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  selectable.control_marker_.controls.push_back(sixDof);

  selectable.control_marker_.controls.push_back(button);
  selectable.control_marker_.description = description;

  selectable.control_marker_.name = name + "_control";
  selectable.selection_marker_.name = name + "_selection";

  selectable.selection_marker_.scale = scale_to_use;
  selectable.control_marker_.scale = scale_to_use;
  (*selectable_objects_)[name] = selectable;
  if(insert_selection) {
    interactive_marker_server_->insert(selectable.selection_marker_, collision_object_selection_feedback_ptr_);
    menu_handler_map_["Collision Object Selection"].apply(*interactive_marker_server_, selectable.selection_marker_.name);
    interactive_marker_server_->applyChanges();
  } 
}

void PlanningSceneEditor::JointControllerCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::string id = "";
  std::string MPR = "";
  PositionType type = StartPosition;

  if(feedback->marker_name.rfind("_start_control") != std::string::npos)
  {
    std::string sub1 = feedback->marker_name.substr(0, feedback->marker_name.rfind("_start_control"));
    id = sub1.substr(0, sub1.rfind("_mpr_"));
    MPR = sub1.substr(sub1.rfind("_mpr_") + 5, sub1.length());
    type = StartPosition;
  }
  else if(feedback->marker_name.rfind("_end_control") != std::string::npos)
  {
    std::string sub1 = feedback->marker_name.substr(0, feedback->marker_name.rfind("_end_control"));
    id = sub1.substr(0, sub1.rfind("_mpr_"));
    MPR = sub1.substr(sub1.rfind("_mpr_") + 5, sub1.length());
    type = GoalPosition;
  }

  if(motion_plan_map_.find(MPR) == motion_plan_map_.end()) {
    ROS_INFO_STREAM("Making mpr in joint controller callback");
  }
  setJointState(motion_plan_map_[MPR], type, id, toBulletTransform(feedback->pose));
}

void PlanningSceneEditor::IKControllerCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::string id = "";
  PositionType type = StartPosition;

  bool findIKSolution = false;
  if(feedback->marker_name.rfind("_start_control") != std::string::npos)
  {
    id = feedback->marker_name.substr(0, feedback->marker_name.rfind("_start_control"));
    type = StartPosition;
  }
  else if(feedback->marker_name.rfind("_end_control") != std::string::npos)
  {
    id = feedback->marker_name.substr(0, feedback->marker_name.rfind("_end_control"));
    type = GoalPosition;
  }
  else
  {
    return;
  }

  IKController& controller = (*ik_controllers_)[id];

  if(motion_plan_map_.find(getMotionPlanRequestNameFromId(controller.motion_plan_id_)) == motion_plan_map_.end()) {
    ROS_INFO_STREAM("Making empty mpr in ik controller callback");
  }

  if(feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE)
  {
    tf::Transform pose = toBulletTransform(feedback->pose);

    if(type == StartPosition)
    {
      motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)].getStartState()->
        updateKinematicStateWithLinkAt(motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)].getEndEffectorLink(),
                                       pose);
      findIKSolution = true;
      if(selected_motion_plan_name_ != getMotionPlanRequestNameFromId(controller.motion_plan_id_))
      {
        selected_motion_plan_name_ = getMotionPlanRequestNameFromId(controller.motion_plan_id_);
        updateState();
      }
    }
    else
    {
      motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)].getGoalState()->
        updateKinematicStateWithLinkAt(motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)].getEndEffectorLink(),
                                       pose);
      findIKSolution = true;
      if(selected_motion_plan_name_ != getMotionPlanRequestNameFromId(controller.motion_plan_id_))
      {
        selected_motion_plan_name_ = getMotionPlanRequestNameFromId(controller.motion_plan_id_);
        updateState();
      }
    }

  }
  else if(feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT)
  {
    if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Map to Robot State"])
    {
      MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)];
      if(data.hasGoodIKSolution(type)) {
        const planning_models::KinematicState* state;
        if(type == StartPosition) {
          state = data.getStartState();
        } else {
          state = data.getGoalState();
        }
        planning_environment::convertKinematicStateToRobotState(*state,
                                                                ros::Time::now(),
                                                                cm_->getWorldFrameId(),
                                                                planning_scene_map_[current_planning_scene_name_].getPlanningScene().robot_state);
        
        sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);
      }
    } else if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Map from Robot State"])
    {
      MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)];

      std::map<std::string, double> vals;
      robot_state_->getKinematicStateValues(vals);

      planning_models::KinematicState* state;
      if(type == StartPosition)
      {
        data.setStartStateValues(vals);
        state = data.getStartState();
        convertKinematicStateToRobotState(*state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                          data.getMotionPlanRequest().start_state);
        data.setLastGoodStartPose((state->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform()));
      }
      else
      {
        state = data.getGoalState();
        data.setGoalStateValues(vals);
        data.setLastGoodGoalPose((state->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform()));
      }
      interactive_marker_server_->setPose(feedback->marker_name, 
                                          toGeometryPose(state->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform()),
                                          feedback->header);
    } else if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Map to Other Orientation"])
    {
      MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)];
      if(type == StartPosition)
      {
        tf::Transform cur_transform = data.getStartState()->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform();
        tf::Transform other_transform = data.getGoalState()->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform();
        cur_transform.setRotation(other_transform.getRotation());
        data.getStartState()->updateKinematicStateWithLinkAt(data.getEndEffectorLink(), cur_transform);
        interactive_marker_server_->setPose(feedback->marker_name, 
                                            toGeometryPose(cur_transform),
                                            feedback->header);
        findIKSolution = true;
      }
      else
      {
        tf::Transform cur_transform = data.getGoalState()->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform();
        tf::Transform other_transform = data.getStartState()->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform();
        cur_transform.setRotation(other_transform.getRotation());
        data.getGoalState()->updateKinematicStateWithLinkAt(data.getEndEffectorLink(), cur_transform);
        interactive_marker_server_->setPose(feedback->marker_name, 
                                            toGeometryPose(cur_transform),
                                            feedback->header);
        findIKSolution = true;
      }
    } else if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Go To Last Good State"])
    {
      MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)];

      if(type == StartPosition)
      {
        data.getStartState()->updateKinematicStateWithLinkAt(data.getEndEffectorLink(), (data.getLastGoodStartPose()));
        interactive_marker_server_->setPose(feedback->marker_name, toGeometryPose(data.getLastGoodStartPose()),
                                            feedback->header);
        findIKSolution = true;
      }
      else
      {
        data.getGoalState()->updateKinematicStateWithLinkAt(data.getEndEffectorLink(), (data.getLastGoodGoalPose()));
        interactive_marker_server_->setPose(feedback->marker_name, toGeometryPose(data.getLastGoodGoalPose()),
                                            feedback->header);
        findIKSolution = true;
      }
    }
    else if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Randomly Perturb"])
    {
      MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)];

      randomlyPerturb(data, type);
      if(type == StartPosition)
      {
        interactive_marker_server_->setPose(feedback->marker_name, toGeometryPose(data.getLastGoodStartPose()),
                                            feedback->header);
      }
      else
      {
        interactive_marker_server_->setPose(feedback->marker_name, toGeometryPose(data.getLastGoodGoalPose()),
                                            feedback->header);
      }
    }
    else if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Plan New Trajectory"])
    {
      MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)];
      unsigned int trajectory;
      planToRequest(data, trajectory);
      selected_trajectory_name_ = getTrajectoryNameFromId(trajectory);
      playTrajectory(data, trajectory_map_[data.getName()][selected_trajectory_name_]);
      updateState();
    }
    else if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Filter Last Trajectory"])
    {
      MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)];
      unsigned int trajectory;
      if(selected_trajectory_name_ != "" && hasTrajectory(data.getName(), selected_trajectory_name_)) {
        filterTrajectory(data, trajectory_map_[data.getName()][selected_trajectory_name_], trajectory);
        selected_trajectory_name_ = getTrajectoryNameFromId(trajectory); 
        playTrajectory(data, trajectory_map_[data.getName()][selected_trajectory_name_]);
        updateState();
      }
    }
    else if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Execute Last Trajectory"])
    {
      std::string trajectory;
      if(selected_trajectory_name_ != "") 
      {
        executeTrajectory(selected_motion_plan_name_, selected_trajectory_name_);
        updateState();
      }
    }
    else if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Delete Request"])
    {
      std::vector<unsigned int> erased_trajectories;
      deleteMotionPlanRequest(controller.motion_plan_id_, erased_trajectories);
    }
  }

  if(findIKSolution)
  {
    if(!solveIKForEndEffectorPose(motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)], type, true))
    {
      if(motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)].hasGoodIKSolution(type))
      {
        motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)].refreshColors();
      }
      motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)].setHasGoodIKSolution(false, type);
    }
    else
    {
      if(!motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)].hasGoodIKSolution(type))
      {
        motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)].refreshColors();
      }
      motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)].setHasGoodIKSolution(true, type);
    }
  }

  if(motion_plan_map_.find(getMotionPlanRequestNameFromId(controller.motion_plan_id_)) == motion_plan_map_.end()) {
    ROS_DEBUG_STREAM("Would be empty mpr in ik controller callback");
  } else if(motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)].areJointControlsVisible())
  {
    createJointMarkers(motion_plan_map_[getMotionPlanRequestNameFromId(controller.motion_plan_id_)], type);
  }
  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::createIkControllersFromMotionPlanRequest(MotionPlanRequestData& data, bool rePose)
{
  if(data.isStartEditable())
  {
    createIKController(data, StartPosition, rePose);
  }

  if(data.isGoalEditable())
  {
    createIKController(data, GoalPosition, rePose);
  }

  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::createIKController(MotionPlanRequestData& data, PositionType type, bool rePose)
{
  KinematicState* state = NULL;
  std::string nametag = "";
  if(type == StartPosition)
  {
    state = data.getStartState();
    nametag = "_start_control";
  }
  else
  {
    state = data.getGoalState();
    nametag = "_end_control";
  }

  tf::Transform transform = state->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform();
  InteractiveMarker marker;


  if(interactive_marker_server_->get(data.getName() + nametag, marker) && rePose)
  {
    geometry_msgs::Pose pose =  toGeometryPose(transform);
    interactive_marker_server_->setPose(data.getName() + nametag, pose);
    return;
  }


  marker.header.frame_id = "/" + cm_->getWorldFrameId();
  marker.pose.position.x = transform.getOrigin().x();
  marker.pose.position.y = transform.getOrigin().y();
  marker.pose.position.z = transform.getOrigin().z();
  marker.pose.orientation.w = transform.getRotation().w();
  marker.pose.orientation.x = transform.getRotation().x();
  marker.pose.orientation.y = transform.getRotation().y();
  marker.pose.orientation.z = transform.getRotation().z();
  marker.scale = 0.225f;
  marker.name = data.getName() + nametag;
  marker.description = data.getName() + nametag;

/*
  InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.always_visible = false;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);
*/

  InteractiveMarkerControl control;
  control.always_visible = false;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  /*control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0;*/
  // changed (Materna)
  control.orientation.w = 0.707;
  control.orientation.x = 0;
  control.orientation.y = -0.707;
  control.orientation.z = 0;

  marker.controls.push_back( control );

  InteractiveMarkerControl control2;
  control2.always_visible = false;
  control2.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  control2.orientation.w = 1;
  control2.orientation.x = 1;
  control2.orientation.y = 0; // it was 1 here (Materna)
  control2.orientation.z = 0;



  Marker marker2;
  marker2.type = Marker::CUBE;
  marker2.scale.x = .2;
  marker2.scale.y = .15;
  marker2.scale.z = .002;
  //marker2.pose.position.x = .1;
  marker2.pose.position.z = 0.1; // added

  // added (Materna)
  marker2.pose.orientation.w = 0.707;
  marker2.pose.orientation.x = 0;
  marker2.pose.orientation.y = -0.707;
  marker2.pose.orientation.z = 0;


  marker2.color.r = 0;
  marker2.color.g = 0;
  marker2.color.b = 0.5;
  marker2.color.a = 1;
  control2.markers.push_back( marker2 );
  marker2.scale.x = .1;
  marker2.scale.y = .35;
  marker2.pose.position.z = 0; // added
  //marker2.pose.position.x = 0;
  //marker2.pose.position.y = 0.1; // added (Materna)
  control2.markers.push_back( marker2 );

  marker.controls.push_back( control2 );

  InteractiveMarkerControl control3;
  control3.always_visible = false;
  control3.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  control3.orientation.w = 1;
  control3.orientation.x = 0;
  control3.orientation.y = 0;
  control3.orientation.z = 1;


  Marker marker3;
  marker3.type = Marker::CUBE;
  marker3.scale.x = .2;
  marker3.scale.y = .002;
  marker3.scale.z = .15;
  //marker3.pose.position.x = .1;
  marker3.pose.position.z = 0.1; // added

  // added (Materna)
  marker3.pose.orientation.w = 0.707;
  marker3.pose.orientation.x = 0;
  marker3.pose.orientation.y = -0.707;
  marker3.pose.orientation.z = 0;

  marker3.color.r = 0;
  marker3.color.g = .5;
  marker3.color.b = 0;
  marker3.color.a = 1;
  control3.markers.push_back( marker3 );
  marker3.scale.x = .1;
  marker3.scale.z = .35;
  marker3.pose.position.z = 0; // added (Materna)
  //marker3.pose.position.x = 0;
  //marker3.pose.position.y = .1; // added (Materna)
  control3.markers.push_back( marker3 );

  marker.controls.push_back( control3 );

  interactive_marker_server_->insert(marker, ik_control_feedback_ptr_);
  control.interaction_mode = InteractiveMarkerControl::MENU;
  //control.markers.push_back(makeMarkerSphere(marker));
  marker.controls.push_back(control);

  (*ik_controllers_)[data.getName()].motion_plan_id_ = data.getId();
  if(type == StartPosition)
  {
    (*ik_controllers_)[data.getName()].start_controller_ = marker;
    data.setLastGoodStartPose(toBulletTransform((*ik_controllers_)[data.getName()].start_controller_.pose));
  }
  else
  {
    (*ik_controllers_)[data.getName()].end_controller_ = marker;
    data.setLastGoodGoalPose(toBulletTransform((*ik_controllers_)[data.getName()].end_controller_.pose));
  }

  menu_handler_map_["IK Control"].apply(*interactive_marker_server_, marker.name);

}

void PlanningSceneEditor::deleteCollisionObject(std::string& name)
{
  (*selectable_objects_)[name].collision_object_.operation.operation
      = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  (*selectable_objects_)[name].attached_collision_object_.object.operation.operation
      = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  interactive_marker_server_->erase((*selectable_objects_)[name].selection_marker_.name);
  interactive_marker_server_->erase((*selectable_objects_)[name].control_marker_.name);
  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::collisionObjectSelectionCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::string name = feedback->marker_name.substr(0, feedback->marker_name.rfind("_selection"));
  bool should_select = false;
  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::BUTTON_CLICK:
      {
        visualization_msgs::InteractiveMarker mark;
        should_select = true;
      }
      break;
    case InteractiveMarkerFeedback::MENU_SELECT:
      if(feedback->menu_entry_id == menu_entry_maps_["Collision Object Selection"]["Delete"])
      {
        deleteCollisionObject(name);
        sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);       
        selectable_objects_->erase(name);
      }
      else if(feedback->menu_entry_id == menu_entry_maps_["Collision Object Selection"]["Select"])
      {
        should_select = true;
      }
      break;
  }

  if(should_select)
  {
    interactive_marker_server_->erase((*selectable_objects_)[name].selection_marker_.name);
    (*selectable_objects_)[name].control_marker_.pose = feedback->pose;
    (*selectable_objects_)[name].control_marker_.header.stamp = ros::Time(ros::WallTime::now().toSec());

    interactive_marker_server_->insert((*selectable_objects_)[name].control_marker_,
                                       collision_object_movement_feedback_ptr_);

    menu_handler_map_["Collision Object"].apply(*interactive_marker_server_,
                                                (*selectable_objects_)[name].control_marker_.name);
  }
  interactive_marker_server_->applyChanges();

}

void PlanningSceneEditor::collisionObjectMovementCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::string name = feedback->marker_name.substr(0, feedback->marker_name.rfind("_control"));

  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::MOUSE_UP:
      if(feedback->control_name == "button") return;
      if(last_resize_handle_ == menu_entry_maps_["Collision Object"]["Off"]) {
        for(size_t i = 0; i < (*selectable_objects_)[name].collision_object_.poses.size(); i++)
        {
          (*selectable_objects_)[name].collision_object_.poses[i] = feedback->pose;
        }
        sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);
      } else {
        CollisionObject coll = (*selectable_objects_)[name].collision_object_;
        tf::Transform orig, cur;
        tf::poseMsgToTF(coll.poses[0], orig);
        tf::poseMsgToTF(feedback->pose, cur);
        tf::Transform nt = orig.inverse()*cur;
        if(last_resize_handle_ == menu_entry_maps_["Collision Object"]["Grow"]) {
          if(coll.shapes[0].type == arm_navigation_msgs::Shape::BOX) {
            coll.shapes[0].dimensions[0] += fabs(nt.getOrigin().x());
            coll.shapes[0].dimensions[1] += fabs(nt.getOrigin().y());
            coll.shapes[0].dimensions[2] += fabs(nt.getOrigin().z());
          } else if(coll.shapes[0].type == arm_navigation_msgs::Shape::CYLINDER) {
            coll.shapes[0].dimensions[0] += fmax(fabs(nt.getOrigin().x()), fabs(nt.getOrigin().y()));
            coll.shapes[0].dimensions[1] += fabs(nt.getOrigin().z());
          } else if(coll.shapes[0].type == arm_navigation_msgs::Shape::SPHERE) {
            coll.shapes[0].dimensions[0] += fmax(fmax(fabs(nt.getOrigin().x()), fabs(nt.getOrigin().y())), fabs(nt.getOrigin().z()));
          }
        } else {
          //shrinking
          if(nt.getOrigin().x() > 0) {
            nt.getOrigin().setX(-nt.getOrigin().x());
          }
          if(nt.getOrigin().y() > 0) {
            nt.getOrigin().setY(-nt.getOrigin().y());
          }
          if(nt.getOrigin().z() > 0) {
            nt.getOrigin().setZ(-nt.getOrigin().z());
          }
          //can't be bigger than the current dimensions
          if(coll.shapes[0].type == arm_navigation_msgs::Shape::BOX) {
            nt.getOrigin().setX(fmax(nt.getOrigin().x(), -coll.shapes[0].dimensions[0]+.01));
            nt.getOrigin().setY(fmax(nt.getOrigin().y(), -coll.shapes[0].dimensions[1]+.01));
            nt.getOrigin().setZ(fmax(nt.getOrigin().z(), -coll.shapes[0].dimensions[2]+.01));
            coll.shapes[0].dimensions[0] += nt.getOrigin().x();
            coll.shapes[0].dimensions[1] += nt.getOrigin().y();
            coll.shapes[0].dimensions[2] += nt.getOrigin().z();
          } else if(coll.shapes[0].type == arm_navigation_msgs::Shape::CYLINDER) {
            nt.getOrigin().setX(fmax(nt.getOrigin().x(), -coll.shapes[0].dimensions[0]+.01));
            nt.getOrigin().setY(fmax(nt.getOrigin().y(), -coll.shapes[0].dimensions[0]+.01));
            nt.getOrigin().setZ(fmax(nt.getOrigin().z(), -coll.shapes[0].dimensions[1]+.01));
            coll.shapes[0].dimensions[0] += fmin(nt.getOrigin().x(), nt.getOrigin().y());
            coll.shapes[0].dimensions[1] += nt.getOrigin().z();
          } else if(coll.shapes[0].type == arm_navigation_msgs::Shape::SPHERE) {
            nt.getOrigin().setX(fmax(nt.getOrigin().x(), -coll.shapes[0].dimensions[0]+.01));
            nt.getOrigin().setY(fmax(nt.getOrigin().y(), -coll.shapes[0].dimensions[0]+.01));
            nt.getOrigin().setZ(fmax(nt.getOrigin().z(), -coll.shapes[0].dimensions[0]+.01));
            coll.shapes[0].dimensions[0] += fmin(fmin(nt.getOrigin().x(), nt.getOrigin().y()), nt.getOrigin().z());
          }
        }
        nt.setOrigin(nt.getOrigin()*.5);
        tf::poseTFToMsg(orig*nt, coll.poses[0]);
        
        createSelectableMarkerFromCollisionObject(coll, coll.id, coll.id, (*selectable_objects_)[name].color_, false);
        (*selectable_objects_)[name].control_marker_.header.stamp = ros::Time(ros::WallTime::now().toSec());
        
        interactive_marker_server_->insert((*selectable_objects_)[name].control_marker_,
                                           collision_object_movement_feedback_ptr_);
        menu_handler_map_["Collision Object"].apply(*interactive_marker_server_,
                                                    (*selectable_objects_)[name].control_marker_.name);
        interactive_marker_server_->applyChanges();
        sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);
      }
      break;
    case InteractiveMarkerFeedback::MOUSE_DOWN:
      if(feedback->control_name == "button") {
        interactive_marker_server_->erase((*selectable_objects_)[name].control_marker_.name);
        (*selectable_objects_)[name].selection_marker_.pose = feedback->pose;
        (*selectable_objects_)[name].selection_marker_.header.stamp = ros::Time(ros::WallTime::now().toSec());
        interactive_marker_server_->insert((*selectable_objects_)[name].selection_marker_,
                                           collision_object_selection_feedback_ptr_);
        menu_handler_map_["Collision Object Selection"].apply(*interactive_marker_server_,
                                                              (*selectable_objects_)[name].selection_marker_.name);
      } 
      break;
    case InteractiveMarkerFeedback::MENU_SELECT:
      if(feedback->menu_entry_id == menu_entry_maps_["Collision Object"]["Delete"])
      {
        deleteCollisionObject(name);
        sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);
        selectable_objects_->erase(name);
      }
      else if(feedback->menu_entry_id == menu_entry_maps_["Collision Object"]["Deselect"])
      {
        interactive_marker_server_->erase((*selectable_objects_)[name].control_marker_.name);
        (*selectable_objects_)[name].selection_marker_.pose = feedback->pose;
        (*selectable_objects_)[name].selection_marker_.header.stamp = ros::Time(ros::WallTime::now().toSec());
        interactive_marker_server_->insert((*selectable_objects_)[name].selection_marker_,
                                           collision_object_selection_feedback_ptr_);
        menu_handler_map_["Collision Object Selection"].apply(*interactive_marker_server_,
                                                              (*selectable_objects_)[name].selection_marker_.name);

      } else if(feedback->menu_entry_id == menu_entry_maps_["Collision Object"]["Off"] || 
                feedback->menu_entry_id == menu_entry_maps_["Collision Object"]["Grow"] ||
                feedback->menu_entry_id == menu_entry_maps_["Collision Object"]["Shrink"]) {  
        menu_handler_map_["Collision Object"].setCheckState(last_resize_handle_, MenuHandler::UNCHECKED);
        last_resize_handle_ = feedback->menu_entry_id;
        menu_handler_map_["Collision Object"].setCheckState(last_resize_handle_, MenuHandler::CHECKED);
        menu_handler_map_["Collision Object"].reApply(*interactive_marker_server_);
      } else if(feedback->menu_entry_id == menu_entry_maps_["Collision Object"]["Attach"]) {
        (*selectable_objects_)[name].control_marker_.pose = feedback->pose;
        attachObjectCallback(name);
      }
      break;
    case InteractiveMarkerFeedback::POSE_UPDATE:
      break;
  }

  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::changeToAttached(const std::string& name)
{
  (*selectable_objects_)[name].selection_marker_.pose = (*selectable_objects_)[name].control_marker_.pose;
  (*selectable_objects_)[name].selection_marker_.description = "attached_"+name;
  interactive_marker_server_->erase((*selectable_objects_)[name].control_marker_.name);
  (*selectable_objects_)[name].selection_marker_.header.stamp = ros::Time(ros::WallTime::now().toSec());
  interactive_marker_server_->insert((*selectable_objects_)[name].selection_marker_,
                                     attached_collision_object_feedback_ptr_);
  menu_handler_map_["Attached Collision Object"].apply(*interactive_marker_server_,
                                                     (*selectable_objects_)[name].selection_marker_.name);
  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::attachedCollisionObjectInteractiveCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::string name = feedback->marker_name.substr(0, feedback->marker_name.rfind("_selection"));

  switch (feedback->event_type)
  {
  case InteractiveMarkerFeedback::BUTTON_CLICK:
    {
      if((*selectable_objects_)[name].selection_marker_.description.empty()) {
        (*selectable_objects_)[name].selection_marker_.description = "attached_"+(*selectable_objects_)[name].collision_object_.id;
      } else {
        (*selectable_objects_)[name].selection_marker_.description = "";
      }
      interactive_marker_server_->insert((*selectable_objects_)[name].selection_marker_,
                                          attached_collision_object_feedback_ptr_);
      std_msgs::Header header;
      header.frame_id = cm_->getWorldFrameId();
      header.stamp = ros::Time::now();
      interactive_marker_server_->setPose(feedback->marker_name,
                                          (*selectable_objects_)[name].collision_object_.poses[0],
                                          header);
      menu_handler_map_["Attached Collision Object"].apply(*interactive_marker_server_,
                                                           (*selectable_objects_)[name].selection_marker_.name);
      interactive_marker_server_->applyChanges();
    }
    break;
  case InteractiveMarkerFeedback::MENU_SELECT:
    if(feedback->menu_entry_id == menu_entry_maps_["Attached Collision Object"]["Detach"])
    {
      (*selectable_objects_)[name].detach_ = true;
      
      (*selectable_objects_)[name].control_marker_.pose = (*selectable_objects_)[name].collision_object_.poses[0];
      (*selectable_objects_)[name].control_marker_.description = (*selectable_objects_)[name].collision_object_.id;
      (*selectable_objects_)[name].selection_marker_.description = "";
      interactive_marker_server_->erase((*selectable_objects_)[name].selection_marker_.name);
      interactive_marker_server_->insert((*selectable_objects_)[name].control_marker_,
                                         collision_object_movement_feedback_ptr_);
      menu_handler_map_["Collision Object"].apply(*interactive_marker_server_,
                                                  (*selectable_objects_)[name].control_marker_.name);
      sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);
      interactive_marker_server_->applyChanges();
    }
    break;
  default:
    break;
  }
}
std::string PlanningSceneEditor::createMeshObject(const std::string& name,
                                                  geometry_msgs::Pose pose,
                                                  const std::string& filename,
                                                  const tf::Vector3& scale,
                                                  std_msgs::ColorRGBA color)
{
  shapes::Mesh* mesh = shapes::createMeshFromFilename(filename, &scale);
  if(mesh == NULL) {
    return "";
  }
  arm_navigation_msgs::Shape object;
  if(!planning_environment::constructObjectMsg(mesh, object)) {
    ROS_WARN_STREAM("Object construction fails");
    return "";
  }
  delete mesh;
  arm_navigation_msgs::CollisionObject collision_object;
  collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  collision_object.header.stamp = ros::Time(ros::WallTime::now().toSec());
  collision_object.header.frame_id = cm_->getWorldFrameId();
  if(name.empty()) {
    collision_object.id = generateNewCollisionObjectId();
  } else {
    collision_object.id = name;
  }
  collision_object.shapes.push_back(object);
  collision_object.poses.push_back(pose);

  lockScene();
  createSelectableMarkerFromCollisionObject(collision_object, collision_object.id, collision_object.id, color);

  ROS_INFO("Created collision object.");
  ROS_INFO("Sending planning scene %s", current_planning_scene_name_.c_str());

  sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);

  unlockScene();
  return collision_object.id;
}

std::string PlanningSceneEditor::createCollisionObject(const std::string& name,
                                                       geometry_msgs::Pose pose, PlanningSceneEditor::GeneratedShape shape,
                                                       float scaleX, float scaleY, float scaleZ, std_msgs::ColorRGBA color)
{
  lockScene();
  arm_navigation_msgs::CollisionObject collision_object;
  collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  collision_object.header.stamp = ros::Time(ros::Time::now().toSec());

  ROS_INFO_ONCE("Using %s as frame_id for collision object.",cm_->getWorldFrameId().c_str());
  collision_object.header.frame_id = cm_->getWorldFrameId();
  //ros::param::param<std::string>("~world_frame",collision_object.header.frame_id,"base_link");
  //cout << "collision_object.header.frame_id==" << collision_object.header.frame_id << endl;;

  if(name.empty()) {
    collision_object.id = generateNewCollisionObjectId();
  } else {
    collision_object.id = name;
  }
  arm_navigation_msgs::Shape object;

  switch (shape)
  {
    case PlanningSceneEditor::Box:
      object.type = arm_navigation_msgs::Shape::BOX;
      object.dimensions.resize(3);
      object.dimensions[0] = scaleX;
      object.dimensions[1] = scaleY;
      object.dimensions[2] = scaleZ;
      break;
    case PlanningSceneEditor::Cylinder:
      object.type = arm_navigation_msgs::Shape::CYLINDER;
      object.dimensions.resize(2);
      object.dimensions[0] = scaleX * 0.5f;
      object.dimensions[1] = scaleZ;
      break;
    case PlanningSceneEditor::Sphere:
      object.type = arm_navigation_msgs::Shape::SPHERE;
      object.dimensions.resize(1);
      object.dimensions[0] = scaleX * 0.5f;
      break;
    default:
      object.type = arm_navigation_msgs::Shape::SPHERE;
      object.dimensions.resize(1);
      object.dimensions[0] = scaleX * 0.5f;
      break;
  };

  collision_object.shapes.push_back(object);
  collision_object.poses.push_back(pose);

  createSelectableMarkerFromCollisionObject(collision_object, collision_object.id, collision_object.id, color);

  ROS_INFO("Created collision object.");
  ROS_INFO("Sending planning scene %s", current_planning_scene_name_.c_str());

  sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);

  unlockScene();
  return collision_object.id;

}

void PlanningSceneEditor::attachCollisionObject(const std::string& name,
                                                const std::string& link_name,
                                                const std::vector<std::string>& touch_links) {
  lockScene();
  if(selectable_objects_->find(name) == selectable_objects_->end()) {
    ROS_WARN("Must already have selectable object to add collision object");
    unlockScene();
    return;
  }

  SelectableObject& selectable = (*selectable_objects_)[name];

  selectable.attached_collision_object_.object = selectable.collision_object_;
  selectable.attached_collision_object_.link_name = link_name;
  selectable.attached_collision_object_.touch_links = touch_links;
  selectable.attached_collision_object_.object.operation.operation = selectable.attached_collision_object_.object.operation.ADD; 
  selectable.attach_ = true;
  //now we need to map the collision object pose into the attached object pose
  geometry_msgs::Pose p = selectable.attached_collision_object_.object.poses[0];
  geometry_msgs::PoseStamped ret_pose;
  ROS_DEBUG_STREAM("Before attach object pose frame " << selectable.attached_collision_object_.object.header.frame_id << " is " 
                  << p.position.x << " " << p.position.y << " " << p.position.z); 
  cm_->convertPoseGivenWorldTransform(*robot_state_,
                                      link_name, 
                                      selectable.attached_collision_object_.object.header,
                                      selectable.attached_collision_object_.object.poses[0],
                                      ret_pose);
  selectable.attached_collision_object_.object.header = ret_pose.header;
  selectable.attached_collision_object_.object.poses[0] = ret_pose.pose;
  ROS_DEBUG_STREAM("Converted attach object pose frame " << ret_pose.header.frame_id << " is " << ret_pose.pose.position.x << " " << ret_pose.pose.position.y << " " << ret_pose.pose.position.z); 
  sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);
  unlockScene();
}

bool PlanningSceneEditor::solveIKForEndEffectorPose(MotionPlanRequestData& data,
                                                    planning_scene_utils::PositionType type, 
                                                    bool coll_aware,
                                                    double change_redundancy)
{
  kinematics_msgs::PositionIKRequest ik_request;
  ik_request.ik_link_name = data.getEndEffectorLink();
  ik_request.pose_stamped.header.frame_id = cm_->getWorldFrameId();
  ik_request.pose_stamped.header.stamp = ros::Time(ros::WallTime::now().toSec());
  
  ROS_DEBUG("Solving IK for %s link in %s world frame id.",ik_request.ik_link_name.c_str(),cm_->getWorldFrameId().c_str());

  KinematicState* state = NULL;
  if(type == StartPosition)
  {
    state = data.getStartState();
  }
  else
  {
    state = data.getGoalState();
  }

  tf::poseTFToMsg(state->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform(), ik_request.pose_stamped.pose);

  convertKinematicStateToRobotState(*state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                    ik_request.robot_state);
  ik_request.ik_seed_state = ik_request.robot_state;

  map<string, double> joint_values;
  vector<string> joint_names;

  if(coll_aware)
  {
    kinematics_msgs::GetConstraintAwarePositionIK::Request ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;
    if(data.hasPathConstraints())
    {
      planning_scene_utils::PositionType other_type;
      if(type == StartPosition)
      {
        other_type = GoalPosition;
      }
      else
      {
        other_type = StartPosition;
      }
      MotionPlanRequest mpr;
      data.setGoalAndPathPositionOrientationConstraints(mpr, other_type);
      mpr.goal_constraints.position_constraints.clear();
        
      arm_navigation_msgs::ArmNavigationErrorCodes err;
      if(!cm_->isKinematicStateValid(*state, std::vector<std::string>(), err, mpr.goal_constraints, mpr.path_constraints))
      {
        ROS_DEBUG_STREAM("Violates rp constraints");
        return false;
      }
      ik_req.constraints = mpr.goal_constraints;
    }
    ik_req.ik_request = ik_request;
    ik_req.timeout = ros::Duration(0.2);
    if(!(*collision_aware_ik_services_)[data.getEndEffectorLink()]->call(ik_req, ik_res))
    {
      ROS_INFO("Problem with ik service call");
      return false;
    }
    if(ik_res.error_code.val != ik_res.error_code.SUCCESS)
    {
      ROS_DEBUG_STREAM("Call yields bad error code " << ik_res.error_code.val);
      return false;
    }
    joint_names = ik_res.solution.joint_state.name;

    for(unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i++)
    {
      joint_values[ik_res.solution.joint_state.name[i]] = ik_res.solution.joint_state.position[i];
    }

  }
  else
  {
    kinematics_msgs::GetPositionIK::Request ik_req;
    kinematics_msgs::GetPositionIK::Response ik_res;
    ik_req.ik_request = ik_request;
    ik_req.timeout = ros::Duration(0.2);
    if(!(*non_collision_aware_ik_services_)[data.getEndEffectorLink()]->call(ik_req, ik_res))
    {
      ROS_INFO("Problem with ik service call");
      return false;
    }
    if(ik_res.error_code.val != ik_res.error_code.SUCCESS)
    {
      ROS_DEBUG_STREAM("Call yields bad error code " << ik_res.error_code.val);
      return false;
    }
    for(unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i++)
    {
      joint_values[ik_res.solution.joint_state.name[i]] = ik_res.solution.joint_state.position[i];
    }

  }

  lockScene();
  state->setKinematicState(joint_values);

  if(coll_aware)
  {
    Constraints emp_con;
    ArmNavigationErrorCodes error_code;

    collision_space::EnvironmentModel::AllowedCollisionMatrix acm = cm_->getCurrentAllowedCollisionMatrix();
    cm_->disableCollisionsForNonUpdatedLinks(data.getGroupName());

    if(!cm_->isKinematicStateValid(*state, joint_names, error_code, emp_con, emp_con, true))
    {
      ROS_INFO_STREAM("Problem with response");
      cm_->setAlteredAllowedCollisionMatrix(acm);
      unlockScene();
      return false;
    }
    cm_->setAlteredAllowedCollisionMatrix(acm);
  }

  if(type == StartPosition)
  {
    data.setStartStateValues(joint_values);
    convertKinematicStateToRobotState(*state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                      data.getMotionPlanRequest().start_state);
    data.setLastGoodStartPose((state->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform()));
  }
  else
  {
    data.setGoalStateValues(joint_values);
    data.setLastGoodGoalPose((state->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform()));
  }
  unlockScene();

  return true;
}

void PlanningSceneEditor::setJointState(MotionPlanRequestData& data, PositionType position, std::string& jointName,
                                        tf::Transform value)
{
  KinematicState* currentState = NULL;

  if(position == StartPosition)
  {
    currentState = data.getStartState();
  }
  else if(position == GoalPosition)
  {
    currentState = data.getGoalState();
  }

  if(currentState == NULL)
  {
    ROS_ERROR("Robot state for request %s is null!", data.getName().c_str());
    return;
  }

  string parentLink = currentState->getKinematicModel()->getJointModel(jointName)->getParentLinkModel()->getName();
  string childLink = currentState->getKinematicModel()->getJointModel(jointName)->getChildLinkModel()->getName();
  KinematicState::JointState* jointState = currentState->getJointState(jointName);
  const KinematicModel::JointModel* jointModel = jointState->getJointModel();

  bool isRotational = (dynamic_cast<const KinematicModel::RevoluteJointModel*> (jointModel) != NULL);
  bool isPrismatic = (dynamic_cast<const KinematicModel::PrismaticJointModel*> (jointModel) != NULL);

  KinematicState::LinkState* linkState = currentState->getLinkState(parentLink);
  tf::Transform transformedValue;

  if(isPrismatic)
  {
    value.setRotation(jointState->getVariableTransform().getRotation());
    transformedValue = currentState->getLinkState(childLink)->getLinkModel()->getJointOriginTransform().inverse()
        * linkState->getGlobalLinkTransform().inverse() * value;
  }
  else if(isRotational)
  {
    transformedValue = currentState->getLinkState(childLink)->getLinkModel()->getJointOriginTransform().inverse()
        * linkState->getGlobalLinkTransform().inverse() * value;
  }

  tf::Transform oldState = jointState->getVariableTransform();
  jointState->setJointStateValues(transformedValue);

  map<string, double> stateMap;
  if(currentState->isJointWithinBounds(jointName))
  {
    currentState->getKinematicStateValues(stateMap);
    currentState->setKinematicState(stateMap);


    // Send state to robot model.
    if(position == StartPosition)
    {
      convertKinematicStateToRobotState(*currentState,
                                        data.getMotionPlanRequest().start_state.joint_state.header.stamp,
                                        data.getMotionPlanRequest().start_state.joint_state.header.frame_id,
                                        data.getMotionPlanRequest().start_state);
    }
    else
    {
      std::vector<JointConstraint>& constraints = data.getMotionPlanRequest().goal_constraints.joint_constraints;

      for(size_t i = 0; i < constraints.size(); i++)
      {
        JointConstraint& constraint = constraints[i];
        constraint.position = stateMap[constraint.joint_name];
      }
    }


    createIKController(data, position, true);
    createJointMarkers(data, position);
  }
  else
  {
    jointState->setJointStateValues(oldState);
  }
}

void PlanningSceneEditor::deleteJointMarkers(MotionPlanRequestData& data, PositionType type)
{
  vector<string> jointNames = data.getJointNamesInGoal();
  for(size_t i = 0; i < jointNames.size(); i++)
  {
    if(type == StartPosition)
    {
      std::string markerName = jointNames[i] + "_mpr_" + data.getName() + "_start_control";

      InteractiveMarker dummy;
      if(interactive_marker_server_->get(markerName, dummy))
      {
        interactive_marker_server_->erase(markerName);
      }
    }
    else
    {
      std::string markerName = jointNames[i] + "_mpr_" + data.getName() + "_end_control";
      InteractiveMarker dummy;
      if(interactive_marker_server_->get(markerName, dummy))
      {
        interactive_marker_server_->erase(markerName);
      }
    }
  }
}

void PlanningSceneEditor::createJointMarkers(MotionPlanRequestData& data, PositionType position)
{
  vector<string> jointNames = data.getJointNamesInGoal();

  KinematicState* state = NULL;
  std::string sauce = "";

  if(position == StartPosition)
  {
    state = data.getStartState();
    sauce = "_start_control";
  }
  else if(position == GoalPosition)
  {
    state = data.getGoalState();
    sauce = "_end_control";
  }

  // For each joint model, find the location of its axis and make a control there.
  for(size_t i = 0; i < jointNames.size(); i++)
  {
    const string& jointName = jointNames[i];
    KinematicModel::JointModel* model =
        (KinematicModel::JointModel*)(state->getKinematicModel()->getJointModel(jointName));

    std::string controlName = jointName + "_mpr_" + data.getName() + sauce;
    joint_clicked_map_[controlName] = false;

    if(model->getParentLinkModel() != NULL)
    {
      string parentLinkName = model->getParentLinkModel()->getName();
      string childLinkName = model->getChildLinkModel()->getName();
      tf::Transform transform = state->getLinkState(parentLinkName)->getGlobalLinkTransform()
          * (state->getKinematicModel()->getLinkModel(childLinkName)->getJointOriginTransform()
              * (state->getJointState(jointName)->getVariableTransform()));

      joint_prev_transform_map_[controlName] = transform;

      InteractiveMarker dummy;
      if(interactive_marker_server_->get(controlName, dummy))
      {
        dummy.header.frame_id = cm_->getWorldFrameId();
        interactive_marker_server_->setPose(controlName, toGeometryPose(transform), dummy.header);
        continue;
      }

      const shapes::Shape* linkShape = model->getChildLinkModel()->getLinkShape();
      const shapes::Mesh* meshShape = dynamic_cast<const shapes::Mesh*> (linkShape);

      KinematicModel::RevoluteJointModel* revoluteJoint = dynamic_cast<KinematicModel::RevoluteJointModel*> (model);
      KinematicModel::PrismaticJointModel* prismaticJoint = dynamic_cast<KinematicModel::PrismaticJointModel*> (model);
      double maxDimension = 0.0f;
      if(meshShape != NULL)
      {

        for(unsigned int i = 0; i < meshShape->vertexCount; i++)
        {
          double x = meshShape->vertices[3 * i];
          double y = meshShape->vertices[3 * i];
          double z = meshShape->vertices[3 * i];

          if(abs(maxDimension) < abs(sqrt(x * x + y * y + z * z)))
          {
            maxDimension = abs(x);
          }

        }

        maxDimension *= 3.0;

        maxDimension = max(0.15, maxDimension);
        maxDimension = min(0.5, maxDimension);
      }
      else
      {
        maxDimension = 0.15;
      }

      if(revoluteJoint != NULL)
      {
        makeInteractive1DOFRotationMarker(transform, revoluteJoint->axis_, controlName, "", (float)maxDimension,
                                          state->getJointState(jointName)->getJointStateValues()[0]);
      }
      else if(prismaticJoint != NULL)
      {
        makeInteractive1DOFTranslationMarker(transform, prismaticJoint->axis_, controlName, "", (float)maxDimension,
                                             state-> getJointState(jointName)->getJointStateValues()[0]);
      }

    }
  }
  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::makeInteractive1DOFTranslationMarker(tf::Transform transform, tf::Vector3 axis, string name,
                                                               string description, float scale, float value)
{
  InteractiveMarker marker;
  marker.header.frame_id = cm_->getWorldFrameId();
  marker.pose.position.x = transform.getOrigin().x();
  marker.pose.position.y = transform.getOrigin().y();
  marker.pose.position.z = transform.getOrigin().z();
  marker.pose.orientation.w = transform.getRotation().w();
  marker.pose.orientation.x = transform.getRotation().x();
  marker.pose.orientation.y = transform.getRotation().y();
  marker.pose.orientation.z = transform.getRotation().z();
  marker.scale = scale;
  marker.name = name;
  marker.description = description;
  InteractiveMarker dummy;
  InteractiveMarkerControl control;
  if(interactive_marker_server_->get(marker.name, dummy))
  {
    interactive_marker_server_->setPose(marker.name, marker.pose, marker.header);
  }
  else
  {
    control.orientation.x = axis.x();
    control.orientation.y = axis.z();
    control.orientation.z = axis.y();
    control.orientation.w = 1;
    control.independent_marker_orientation = false;
    control.always_visible = false;
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);
    interactive_marker_server_->insert(marker);
    interactive_marker_server_->setCallback(marker.name, joint_control_feedback_ptr_);
  }

}

void PlanningSceneEditor::makeInteractive1DOFRotationMarker(tf::Transform transform, tf::Vector3 axis, string name,
                                                            string description, float scale, float angle)
{
  InteractiveMarker marker;
  marker.header.frame_id = cm_->getWorldFrameId();
  marker.pose.position.x = transform.getOrigin().x();
  marker.pose.position.y = transform.getOrigin().y();
  marker.pose.position.z = transform.getOrigin().z();
  marker.pose.orientation.w = transform.getRotation().w();
  marker.pose.orientation.x = transform.getRotation().x();
  marker.pose.orientation.y = transform.getRotation().y();
  marker.pose.orientation.z = transform.getRotation().z();
  marker.scale = scale;
  marker.name = name;
  marker.description = description;

  InteractiveMarker dummy;
  if(interactive_marker_server_->get(marker.name, dummy))
  {
    interactive_marker_server_->setPose(marker.name, marker.pose, marker.header);
  }
  else
  {
    InteractiveMarkerControl control;
    control.orientation.x = axis.x();
    control.orientation.y = axis.z();
    control.orientation.z = axis.y();
    control.orientation.w = 1;
    control.independent_marker_orientation = false;
    control.always_visible = false;
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    interactive_marker_server_->insert(marker);
    interactive_marker_server_->setCallback(marker.name, joint_control_feedback_ptr_);
  }
}

void PlanningSceneEditor::setIKControlsVisible(std::string id, PositionType type, bool visible)
{
  if(!visible)
  {
    if(type == StartPosition)
    {
      interactive_marker_server_->erase((*ik_controllers_)[id].start_controller_.name);
    }
    else
    {
      interactive_marker_server_->erase((*ik_controllers_)[id].end_controller_.name);
    }
    interactive_marker_server_->applyChanges();
  }
  else
  {
    createIKController(motion_plan_map_[id], type, false);
    interactive_marker_server_->applyChanges();
  }
}

void PlanningSceneEditor::executeTrajectory(TrajectoryData& trajectory)
{
  // if(params_.sync_robot_state_with_gazebo_)
  // {
  //   pr2_mechanism_msgs::ListControllers listControllers;

  //   if(!list_controllers_client_.call(listControllers.request, listControllers.response))
  //   {
  //     ROS_ERROR("Failed to get list of controllers!");
  //     return;
  //   }

  //   std::map<std::string, planning_models::KinematicModel::JointModelGroup*> jointModelGroupMap =
  //       cm_->getKinematicModel()->getJointModelGroupMap();
  //   planning_models::KinematicModel::JointModelGroup* rightGroup = NULL;
  //   planning_models::KinematicModel::JointModelGroup* leftGroup = NULL;
  //   if(params_.right_arm_group_ != "none")
  //   {
  //     rightGroup = jointModelGroupMap[params_.right_arm_group_];
  //   }

  //   if(params_.left_arm_group_ != "none")
  //   {
  //     leftGroup = jointModelGroupMap[params_.left_arm_group_];
  //   }

  //   pr2_mechanism_msgs::SwitchController switchControllers;
  //   switchControllers.request.stop_controllers = listControllers.response.controllers;
  //   if(!switch_controllers_client_.call(switchControllers.request, switchControllers.response))
  //   {
  //     ROS_ERROR("Failed to shut down controllers!");
  //     return;
  //   }

  //   ROS_INFO("Shut down controllers.");

  //   MotionPlanRequestData& motionPlanData = motion_plan_map_[getMotionPlanRequestNameFromId(trajectory.getMotionPlanRequestId())];

  //   gazebo_msgs::SetModelConfiguration modelConfiguration;
  //   modelConfiguration.request.model_name = params_.gazebo_model_name_;
  //   modelConfiguration.request.urdf_param_name = params_.robot_description_param_;

  //   for(size_t i = 0; i < motionPlanData.getStartState()->getJointStateVector().size(); i++)
  //   {
  //     const KinematicState::JointState* jointState = motionPlanData.getStartState()->getJointStateVector()[i];
  //     if(jointState->getJointStateValues().size() > 0)
  //     {
  //       modelConfiguration.request.joint_names.push_back(jointState->getName());
  //       modelConfiguration.request.joint_positions.push_back(jointState->getJointStateValues()[0]);
  //     }
  //   }

  //   if(!gazebo_joint_state_client_.call(modelConfiguration.request, modelConfiguration.response))
  //   {
  //     ROS_ERROR("Failed to call gazebo set joint state client!");
  //     return;
  //   }

  //   ROS_INFO("Set joint state");

  //   if(!modelConfiguration.response.success)
  //   {
  //     ROS_ERROR("Failed to set gazebo model configuration to start state!");
  //     return;
  //   }
  //   ROS_INFO("Gazebo returned: %s", modelConfiguration.response.status_message.c_str());

  //   pr2_mechanism_msgs::SwitchController restartControllers;
  //   restartControllers.request.start_controllers = listControllers.response.controllers;
  //   if(!switch_controllers_client_.call(restartControllers.request, restartControllers.response))
  //   {
  //     ROS_ERROR("Failed to restart controllers: service call failed!");
  //     return;
  //   }
  //   else if(!restartControllers.response.ok)
  //   {
  //     ROS_ERROR("Failed to restart controllers: Response not ok!");
  //   }

  //   ROS_INFO("Restart controllers.");

  //   ros::Time::sleepUntil(ros::Time::now() + ros::Duration(0.5));
  //   SimpleActionClient<FollowJointTrajectoryAction>* controller = arm_controller_map_[trajectory.getGroupName()];
  //   FollowJointTrajectoryGoal goal;
  //   goal.trajectory.joint_names = trajectory.getTrajectory().joint_names;
  //   goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);
  //   trajectory_msgs::JointTrajectoryPoint endPoint = trajectory.getTrajectory().points[0];
  //   endPoint.time_from_start = ros::Duration(1.0);
  //   goal.trajectory.points.push_back(endPoint);
  //   controller->sendGoalAndWait(goal, ros::Duration(1.0), ros::Duration(1.0));
  //   ros::Time::sleepUntil(ros::Time::now() + ros::Duration(1.0));

  // }

  SimpleActionClient<FollowJointTrajectoryAction>* controller = arm_controller_map_[trajectory.getGroupName()];
  FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory.getTrajectory();
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);
  controller->sendGoal(goal, boost::bind(&PlanningSceneEditor::controllerDoneCallback, this, _1, _2));
  logged_group_name_ = trajectory.getGroupName();
  logged_motion_plan_request_ = getMotionPlanRequestNameFromId(trajectory.getMotionPlanRequestId());
  logged_trajectory_ = trajectory.getTrajectory();
  logged_trajectory_.points.clear();
  logged_trajectory_controller_error_.points.clear();
  logged_trajectory_start_time_ = ros::Time::now() + ros::Duration(0.2);
  monitor_status_ = Executing;

}

void PlanningSceneEditor::randomlyPerturb(MotionPlanRequestData& mpr, PositionType type)
{
  lockScene();

  //Joint space method

  KinematicState* currentState = NULL;

  if(type == StartPosition)
  {
    currentState = mpr.getStartState();
  }
  else
  {
    currentState = mpr.getGoalState();
  }

  vector<KinematicState::JointState*>& jointStates = currentState->getJointStateVector();
  std::map<string, double> originalState;
  std::map<string, double> stateMap;
  bool goodSolution = false;
  int numIterations = 0;
  int maxIterations = 100;
  while(!goodSolution && numIterations < maxIterations)
  {
    currentState->getKinematicStateValues(stateMap);
    for(size_t i = 0; i < jointStates.size(); i++)
    {
      KinematicState::JointState* jointState = jointStates[i];
      map<string, pair<double, double> > bounds = jointState->getJointModel()->getAllVariableBounds();
      for(map<string, pair<double, double> >::iterator it = bounds.begin(); it != bounds.end(); it++)
      {
        if(!mpr.isJointNameInGoal(it->first))
        {
          continue;
        }
        double range = it->second.second - it->second.first;
        if(range == std::numeric_limits<double>::infinity())
        {
          continue;
        }
        double randVal = ((double)random() / (double)RAND_MAX) * (range * 0.99) + it->second.first;
        stateMap[it->first] = randVal;
      }
    }

    currentState->setKinematicState(stateMap);

    if(!cm_->isKinematicStateInCollision(*currentState))
    {
      goodSolution = true;
      break;
    }
    numIterations++;
  }

  if(!goodSolution)
  {
    currentState->setKinematicState(originalState);
    unlockScene();
    return;
  }
  else
  {
    ROS_INFO("Found a good random solution in %d iterations", numIterations);
  }

  if(type == StartPosition)
  {
    convertKinematicStateToRobotState(*currentState, mpr.getMotionPlanRequest().start_state.joint_state.header.stamp,
                                      mpr.getMotionPlanRequest().start_state.joint_state.header.frame_id,
                                      mpr.getMotionPlanRequest().start_state);
  }
  else
  {
    std::vector<JointConstraint>& constraints = mpr.getMotionPlanRequest().goal_constraints.joint_constraints;
    for(size_t i = 0; i < constraints.size(); i++)
    {
      JointConstraint& constraint = constraints[i];
      constraint.position = stateMap[constraint.joint_name];
    }
  }
  mpr.setHasGoodIKSolution(true, type);
  createIKController(mpr, type, false);
  mpr.setJointControlsVisible(mpr.areJointControlsVisible(), this);
  interactive_marker_server_->applyChanges();
  mpr.refreshColors();
  unlockScene();

}

void PlanningSceneEditor::controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                                 const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  MotionPlanRequestData& mpr = motion_plan_map_[logged_motion_plan_request_];
  TrajectoryData logged(mpr.getNextTrajectoryId(), "Robot Monitor", logged_group_name_, logged_trajectory_);
  logged.setTrajectoryError(logged_trajectory_controller_error_);
  logged.setBadPoint(-1);
  logged.setDuration(ros::Time::now() - logged_trajectory_start_time_);
  logged.setTrajectoryRenderType(Temporal);
  logged.setMotionPlanRequestId(mpr.getId());
  logged.trajectory_error_code_.val = result->error_code;
  mpr.addTrajectoryId(logged.getId());
  trajectory_map_[mpr.getName()][logged.getName()] = logged;
  logged_trajectory_.points.clear();
  logged_trajectory_controller_error_.points.clear();
  //logged_group_name_ = "";
  //logged_motion_plan_request_ = "";
  selected_trajectory_name_ = getTrajectoryNameFromId(logged.getId());
  updateState();
  ROS_INFO("CREATING TRAJECTORY %s", logged.getName().c_str());

  // Keep recording a second trajectory to analize the overshoot
  monitor_status_ = WaitingForStop;
  time_of_controller_done_callback_ = ros::Time::now();
  time_of_last_moving_notification_ = ros::Time::now();
  logged_trajectory_start_time_ = ros::Time::now();
}

void PlanningSceneEditor::armHasStoppedMoving()
{
  MotionPlanRequestData& mpr = motion_plan_map_[logged_motion_plan_request_];
  TrajectoryData logged(mpr.getNextTrajectoryId(), "Overshoot Monitor", logged_group_name_, logged_trajectory_);
  logged.setTrajectoryError(logged_trajectory_controller_error_);
  logged.setBadPoint(-1);
  logged.setDuration(ros::Duration(0));
  logged.setTrajectoryRenderType(Temporal);
  logged.setMotionPlanRequestId(mpr.getId());
  logged.trajectory_error_code_.val = 0;
  mpr.addTrajectoryId(logged.getId());
  trajectory_map_[mpr.getName()][logged.getName()] = logged;
  logged_trajectory_.points.clear();
  logged_trajectory_controller_error_.points.clear();
  logged_group_name_ = "";
  logged_motion_plan_request_ = "";
  //selected_trajectory_name_ = getTrajectoryNameFromId(logged.getId());
  updateState();
  ROS_INFO("CREATING TRAJECTORY %s", logged.getName().c_str());

  monitor_status_ = idle;
}

void PlanningSceneEditor::getAllRobotStampedTransforms(const planning_models::KinematicState& state,
                                  vector<geometry_msgs::TransformStamped>& trans_vector, const ros::Time& stamp)
{
  trans_vector.clear();
  const map<string, geometry_msgs::TransformStamped>& transforms = cm_->getSceneTransformMap();
  geometry_msgs::TransformStamped transvec;
  for(map<string, geometry_msgs::TransformStamped>::const_iterator it = transforms.begin(); it
      != transforms.end(); it++)
  {
    if(it->first != cm_->getWorldFrameId())
    {
      trans_vector.push_back(it->second);
    }
  }
  for(unsigned int i = 0; i < state.getLinkStateVector().size(); i++)
  {
    const planning_models::KinematicState::LinkState* ls = state.getLinkStateVector()[i];

    if(ls->getName() != cm_->getWorldFrameId())
    {
      geometry_msgs::TransformStamped ts;
      ts.header.stamp = stamp;
      ts.header.frame_id = cm_->getWorldFrameId();

      ts.child_frame_id = ls->getName();
      tf::transformTFToMsg(ls->getGlobalLinkTransform(), ts.transform);
      trans_vector.push_back(ts);
    }
  }
}

void PlanningSceneEditor::sendTransformsAndClock()
{
  if(robot_state_ == NULL)
  {
    return;
  }

  if(!params_.use_robot_data_)
  {
    ros::WallTime cur_time = ros::WallTime::now();
    rosgraph_msgs::Clock c;
    c.clock.nsec = cur_time.nsec;
    c.clock.sec = cur_time.sec;
    //clock_publisher_.publish(c);


    getAllRobotStampedTransforms(*robot_state_, robot_transforms_, c.clock);
    transform_broadcaster_.sendTransform(robot_transforms_);
  }
}

MenuHandler::EntryHandle PlanningSceneEditor::registerSubMenuEntry(string menu, string name, string subMenu,
                                                                   MenuHandler::FeedbackCallback& callback)
{

  MenuHandler::EntryHandle toReturn = menu_handler_map_[menu].insert(menu_entry_maps_[menu][subMenu], name, callback);
  menu_entry_maps_[menu][name] = toReturn;
  return toReturn;
}

MenuHandler::EntryHandle PlanningSceneEditor::registerMenuEntry(string menu, string entryName,
                                                                MenuHandler::FeedbackCallback& callback)
{
  MenuHandler::EntryHandle toReturn = menu_handler_map_[menu].insert(entryName, callback);
  menu_entry_maps_[menu][entryName] = toReturn;
  return toReturn;
}

void PlanningSceneEditor::deleteTrajectory(unsigned int mpr_id, unsigned int traj_id)
{
  if(!hasTrajectory(getMotionPlanRequestNameFromId(mpr_id),
                    getTrajectoryNameFromId(traj_id))) {
    ROS_WARN_STREAM("No trajectory " << traj_id << " in trajectories for " << mpr_id << " for deletion");
    return;
  }
  
  if(current_planning_scene_name_ == "") {
    ROS_WARN_STREAM("Shouldn't be calling without a planning scene");
    return;
  }

  lockScene();
  
  if(motion_plan_map_.find(getMotionPlanRequestNameFromId(mpr_id))
     == motion_plan_map_.end()) {
    ROS_WARN_STREAM("Can't find mpr id " << mpr_id);
    unlockScene();
    return;
  }

  MotionPlanRequestData& request_data = motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)];
  
  if(!request_data.hasTrajectoryId(traj_id)) {
    ROS_WARN_STREAM("Motion plan request " << mpr_id << " doesn't have trajectory id " << traj_id << " for deletion");
    unlockScene();
    return;
  }
  request_data.removeTrajectoryId(traj_id);

  TrajectoryData& traj = trajectory_map_[getMotionPlanRequestNameFromId(mpr_id)][getTrajectoryNameFromId(traj_id)];
    
  for(size_t i = 0; i < states_.size(); i++)
  {
    if(states_[i].state == traj.getCurrentState())
    {
      states_[i].state = NULL;
      states_[i].source = "Delete trajectory";
    }
  }

  traj.reset();
  trajectory_map_[getMotionPlanRequestNameFromId(mpr_id)].erase(getTrajectoryNameFromId(traj_id));
  if(trajectory_map_[getMotionPlanRequestNameFromId(mpr_id)].empty()) {
    trajectory_map_.erase(getMotionPlanRequestNameFromId(mpr_id));
  }

  unlockScene();
  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::deleteMotionPlanRequest(const unsigned int& id,
                                                  std::vector<unsigned int>& erased_trajectories)
{
  if(motion_plan_map_.find(getMotionPlanRequestNameFromId(id)) == motion_plan_map_.end())
  {
    ROS_WARN_STREAM("Trying to delete non-existent motion plan request " << id);
    return;
  }  
  lockScene();
  MotionPlanRequestData& motion_plan_data = motion_plan_map_[getMotionPlanRequestNameFromId(id)];
  for(size_t i = 0; i < states_.size(); i++)
  {
    if(states_[i].state == motion_plan_data.getStartState() || states_[i].state
       == motion_plan_data.getGoalState())
    {
      states_[i].state = NULL;
      states_[i].source = "Delete motion plan request";
    }
  }
  erased_trajectories.clear();
  for(std::set<unsigned int>::iterator it = motion_plan_data.getTrajectories().begin();
      it != motion_plan_data.getTrajectories().end();
      it++) {
    erased_trajectories.push_back(*it);
  }
  
  for(size_t i = 0; i < erased_trajectories.size(); i++)
  {
    deleteTrajectory(motion_plan_data.getId(), erased_trajectories[i]);
  }
  
  deleteJointMarkers(motion_plan_data, StartPosition);
  deleteJointMarkers(motion_plan_data, GoalPosition);
  interactive_marker_server_->erase(motion_plan_data.getName() + "_start_control");
  interactive_marker_server_->erase(motion_plan_data.getName() + "_end_control");

  motion_plan_data.reset();
  motion_plan_map_.erase(getMotionPlanRequestNameFromId(id));

  if(current_planning_scene_name_ == "") {
    ROS_WARN_STREAM("Shouldn't be trying to delete an MPR without a current planning scene");
  } else {
    PlanningSceneData& data = planning_scene_map_[current_planning_scene_name_];
    if(!data.hasMotionPlanRequestId(id)) {
      ROS_WARN_STREAM("Planning scene " << data.getId() << " doesn't have mpr id " << id << " for delete");
    } else {
      data.removeMotionPlanRequestId(id);
    }
  }
  updateState();
  unlockScene();
  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::executeTrajectory(const std::string& mpr_name,
                                            const std::string& traj_name)
{
  TrajectoryData& traj = trajectory_map_[mpr_name][traj_name];
  executeTrajectory(traj);
}

bool PlanningSceneEditor::hasTrajectory(const std::string& mpr_name, 
                                        const std::string& traj_name) const {
  if(trajectory_map_.find(mpr_name) == trajectory_map_.end()) {
    return false;
  }
  
  if(trajectory_map_.at(mpr_name).find(traj_name) == trajectory_map_.at(mpr_name).end()) {
    return false;
  }
  return true;
}
