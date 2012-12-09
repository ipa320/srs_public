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

/* \author: Matthew Klingensmith */
#ifndef MOVE_ARM_UTILS_H
#define MOVE_ARM_UTILS_H
#include <planning_environment/models/collision_models.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <planning_environment/models/model_utils.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_broadcaster.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <arm_navigation_msgs/GetStateValidity.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <arm_navigation_msgs/convert_messages.h>
//#include <move_arm_warehouse/move_arm_warehouse_logger_reader.h>
#include <srs_assisted_arm_navigation/move_arm_utils/move_arm_warehouse_logger_reader.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/tools.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <planning_environment/monitors/kinematic_model_state_monitor.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <gazebo_msgs/SetModelConfiguration.h>
#include <std_srvs/Empty.h>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/UnloadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
// #include <gazebo_msgs/SetLinkProperties.h>
// #include <gazebo_msgs/GetLinkProperties.h>

typedef map<std::string, interactive_markers::MenuHandler::EntryHandle> MenuEntryMap;
typedef map<std::string, MenuEntryMap> MenuMap;
typedef map<std::string, interactive_markers::MenuHandler> MenuHandlerMap;

////
/// Namespace planning_scene_utils
/// @brief contains utilities for editing planning scenes
////
namespace planning_scene_utils
{

inline static geometry_msgs::Pose toGeometryPose(tf::Transform transform)
{
  geometry_msgs::Pose toReturn;
  toReturn.position.x = transform.getOrigin().x();
  toReturn.position.y = transform.getOrigin().y();
  toReturn.position.z = transform.getOrigin().z();
  toReturn.orientation.x = transform.getRotation().x();
  toReturn.orientation.y = transform.getRotation().y();
  toReturn.orientation.z = transform.getRotation().z();
  toReturn.orientation.w = transform.getRotation().w();
  return toReturn;
}

inline static tf::Transform toBulletTransform(geometry_msgs::Pose pose)
{
  tf::Quaternion quat =
    tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Vector3 vec = tf::Vector3(pose.position.x, pose.position.y, pose.position.z);
  return tf::Transform(quat, vec);
}

inline static std::string getPlanningSceneNameFromId(const unsigned int id) {
  std::stringstream ss;
  ss << "Planning Scene " << id;
  return ss.str();
}

inline static unsigned int getPlanningSceneIdFromName(const std::string& name) {
  std::stringstream ss(name);
  std::string temp;
  ss >> temp;
  ss >> temp;
  unsigned int ret;
  ss >> ret;
  return ret;
}

inline static std::string getMotionPlanRequestNameFromId(const unsigned int id) {
  std::stringstream ss;
  ss << "MPR " << id;
  return ss.str();
}

inline static std::string getTrajectoryNameFromId(const unsigned int id) {
  std::stringstream ss;
  ss << "Trajectory " << id;
  return ss.str();
}

/**
  @brief Convert a control error code into a string value
  @param error_code The input error code
  @return The resultant string message
*/
inline static std::string getResultErrorFromCode(int error_code)
{
  std::string result;
  if(error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
     result = "Success";
  else if(error_code == control_msgs::FollowJointTrajectoryResult::INVALID_GOAL)
     result = "Invalid Goal";
  else if(error_code == control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS)
     result = "Invalid Joints";
  else if(error_code == control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP)
     result = "Old header timestamp";
  else if(error_code == control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED)
     result = "Path tolerance violated";
  else if(error_code == control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED)
     result = "Goal tolerance violated";
  return result;
}

////
/// Enum PositionType
/// @brief Specifies either the start or end position
/// of a motion plan request.
////
enum PositionType
  {
    StartPosition, GoalPosition
  };

////
/// Enum RenderType
/// @brief Specifies how a set of links should be rendered.
/// CollisionMesh: Mesh resource in URDF file listed for testing collisions.
/// VisualMesh: Mesh resource in URDF file listed for visualization.
/// PaddingMesh: Wireframe mesh representing the link's configuration space padding.
/////
enum RenderType
  {
    CollisionMesh, VisualMesh, PaddingMesh
  };

////
/// Enum TrajectoryRenderType
/// @brief Specifies how a trajectory should be rendered.
///  Kinematic: Trajectories are rendered by iterating through the trajectory points (ignoring timestamps).
///  Temporal:  Trajectories that have valid timestamps, are rendered based on the timestamps.
////
enum TrajectoryRenderType
{
  Kinematic,
  Temporal,
};


// Must be defined so that subsequent classes can reference.
class PlanningSceneEditor;

////
/// Class PlanningSceneData
/// @brief Convenience class wrapping a planning scene message
/// and its meta-data.
////
class PlanningSceneData
{
protected:
  std::string name_;
  unsigned int id_;
  std::string host_;
  ros::Time timestamp_;
  arm_navigation_msgs::PlanningScene planning_scene_;
  std::vector<std::string> pipeline_stages_;
  std::vector<arm_navigation_msgs::ArmNavigationErrorCodes> error_codes_;
  std::set<unsigned int> motion_plan_requests_;

public:
  PlanningSceneData();
  PlanningSceneData(unsigned int id, const ros::Time& timestamp, const arm_navigation_msgs::PlanningScene& scene);

  /// @brief Returns the host of the network machine that the planning scene was recorded on.
  inline std::string getHostName() const
  {
    return host_;
  }

  /// @brief Set the host of the network machine that the planning scene was recorded on.
  inline void setHostName(std::string host)
  {
    host_ = host;
  }

  /// @brief Returns the unique Id of the planning scene
  inline std::string getName() const
  {
    return name_;
  }

  inline unsigned int getId() const 
  {
    return id_;
  }

  /// @brief Returns the time that the planning scene was recorded (ignores sim time)
  inline const ros::Time& getTimeStamp() const
  {
    return timestamp_;
  }

  /// @brief Returns the underlying planning scene message.
  inline arm_navigation_msgs::PlanningScene& getPlanningScene()
  {
    return planning_scene_;
  }

  /// @brief Sets the time that the planning scene was recorded
  inline void setTimeStamp(const ros::Time& time)
  {
    timestamp_ = time;
    planning_scene_.robot_state.joint_state.header.stamp = time;
  }

  inline void setId(unsigned int id) {
    id_ = id;
    name_ = getPlanningSceneNameFromId(id);
  }

  /// @brief Sets the underlying planning scene message.
  inline void setPlanningScene(const arm_navigation_msgs::PlanningScene& scene)
  {
    planning_scene_ = scene;
    timestamp_ = scene.robot_state.joint_state.header.stamp;
  }

  /// @brief Returns a vector of arbitrary strings corresponding to the different "pipelines" of the planning
  /// scene. These are defined by the user of the warehouse logger reader. They may be things like "planner"
  /// "filter", "IK", etc.
  inline std::vector<std::string>& getPipelineStages()
  {
    return pipeline_stages_;
  }

  /// @brief see getPipelineStages
  inline void setPipelineStages(std::vector<std::string>& stages)
  {
    pipeline_stages_ = stages;
  }

  /// @brief Gets a vector of trajectory error codes corresponding 1-1 to each pipeline stage
  inline std::vector<arm_navigation_msgs::ArmNavigationErrorCodes>& getErrorCodes()
  {
    return error_codes_;
  }

  /// @brief see getErrorCodes
  inline void setErrorCodes(std::vector<arm_navigation_msgs::ArmNavigationErrorCodes>& error_codes)
  {
    error_codes_ = error_codes;
  }

  /// @brief Returns a vector of Motion Plan Ids of the motion plan requests stored in the planning scene.
  inline std::set<unsigned int>& getRequests()
  {
    return motion_plan_requests_;
  }

  /// @brief Fills the Kinematic state passed into the function with the values specified by
  /// the robot state message inside the underlying planning scene message.
  void getRobotState(planning_models::KinematicState* state);

  bool hasMotionPlanRequestId(unsigned int id) const {
    return(motion_plan_requests_.find(id) != motion_plan_requests_.end());
  }
  
  void addMotionPlanRequestId(unsigned int id) {
    motion_plan_requests_.insert(id);
  }

  void removeMotionPlanRequestId(unsigned int id) {
    motion_plan_requests_.erase(id);
  }

  unsigned int getNextMotionPlanRequestId() const {
    if(motion_plan_requests_.empty()) {
      return 0;
    }
    return (*motion_plan_requests_.rbegin())+1;
  }
};

////
/// Class MotionPlanRequestData
/// @brief Convenience class wrapping a motion plan request
/// and its meta-data
////
class MotionPlanRequestData
{
protected:
  std::string name_;
  unsigned int id_;
  std::string source_;
  unsigned int planning_scene_id_;
  std::string end_effector_link_;
  std::string group_name_;
  arm_navigation_msgs::MotionPlanRequest motion_plan_request_;
  bool is_start_editable_;
  bool is_goal_editable_;
  bool is_start_visible_;
  bool is_goal_visible_;
  bool should_refresh_colors_;
  bool has_refreshed_colors_;
  bool has_path_constraints_;
  bool has_good_goal_ik_solution_;
  bool has_good_start_ik_solution_;
  bool are_collisions_visible_;
  bool has_state_changed_;
  bool are_joint_controls_visible_;

  double roll_tolerance_;
  double pitch_tolerance_;
  double yaw_tolerance_;

  bool constrain_roll_;
  bool constrain_pitch_;
  bool constrain_yaw_;

  std_msgs::ColorRGBA start_color_;
  std_msgs::ColorRGBA goal_color_;
  std::set<unsigned int> trajectories_;
  planning_models::KinematicState* start_state_;
  planning_models::KinematicState* goal_state_;
  tf::Transform last_good_start_pose_;
  tf::Transform last_good_goal_pose_;
  visualization_msgs::MarkerArray collision_markers_;
  RenderType render_type_;
  unsigned int next_trajectory_id_;

public:
  MotionPlanRequestData()
  {
    start_state_ = NULL;
    goal_state_ = NULL;
  }

  MotionPlanRequestData(const planning_models::KinematicState* robot_state);
  MotionPlanRequestData(const unsigned int& id, 
                        const std::string& source, 
                        const arm_navigation_msgs::MotionPlanRequest& request,
                        const planning_models::KinematicState* robot_state, 
                        const std::string& end_effector_link);

  /// @brief If the color of the motion plan request changes, this counter is incremented until it reaches
  /// a value specified by the planning scene editor. This is done to allow the display markers time to disappear
  /// before their colors are changed.
  ros::Duration refresh_timer_;


  /// @brief Sets the start state joint values of the robot.
  /// @param joint_values a map of joint names to values.
  void setStartStateValues(std::map<std::string, double>& joint_values);

  /// @brief Sets the goal state joint values of the robot.
  /// @param joint_values a map of joint names to values.
  void setGoalStateValues(std::map<std::string, double>& joint_values);

  /// @brief Updates the KinematicState pointer start_state_ to reflect changes to the underlying
  /// motion plan request message.
  void updateStartState();

  /// @brief Updates the KinematicState pointer goal_state_ to reflect changes to the underlying
  /// motion plan request message.
  void updateGoalState();

  /// @brief Returns a vector of joint messages corresponding to all the joints in the goal constraints
  /// of the underlying motion plan request message.
  std::vector<std::string> getJointNamesInGoal();

  /// @brief returns whether or not the specified joint name is part of the goal constraints of the motion plan
  /// request.
  /// @param joint a valid joint name.
  /// @return true if the joint with the specified name is in the goal constraints, and false otherwise
  bool isJointNameInGoal(std::string joint);

  /// @brief Gets what mesh to display in RVIZ.
  inline RenderType getRenderType() const
  {
    return render_type_;
  }

  /// @brief Sets what mesh to display in RVIZ.
  inline void setRenderType(RenderType renderType)
  {
    render_type_ = renderType;
  }

  /// @brief returns whether the interactive joint control markers are visible.
  inline bool areJointControlsVisible() const
  {
    return are_joint_controls_visible_;
  }

  /// @brief Either creates or destroys the interactive joint control markers for this request.
  /// @param visible if true, creates the joint markers. If False, destroys them.
  /// @param editor pointer to the planning scene editor responsible for maintaining the markers.
  void setJointControlsVisible(bool visible, PlanningSceneEditor* editor);

  /// @brief Returns true if the joint values or the color of the motion plan request has changed.
  inline bool hasStateChanged() const
  {
    return has_state_changed_;
  }

  /// @brief Set the flag recording whether or not the joint state or color of the motion plan request has changed.
  inline void setStateChanged(bool changed)
  {
    has_state_changed_ = changed;
  }

  /// @brief Each motion plan request stores an array of small red spheres corresponding to collision points.
  inline visualization_msgs::MarkerArray& getCollisionMarkers()
  {
    return collision_markers_;
  }

  /// @brief Return whether or not the small red spheres corresponding to collision points are being published.
  inline bool areCollisionsVisible() const
  {
    return are_collisions_visible_;
  }

  /// @brief Either show or hide the red spheres corresponding to collision points.
  inline void setCollisionsVisible(bool visible)
  {
    are_collisions_visible_ = visible;
  }

  /// @brief Convenience shorthand for setCollisionsVisible(true)
  inline void showCollisions() 
  {
    setCollisionsVisible(true);
  }

  /// @brief Convenience shorthand for setCollisionsVisible(false)
  inline void hideCollisions()
  {
    setCollisionsVisible(false);
  }

  /// @brief Returns the last starting pose that had a good IK solution
  inline tf::Transform getLastGoodStartPose() const
  {
    return last_good_start_pose_;
  }

  /// @brief Returns the last goal pose that had a good IK solution
  inline tf::Transform getLastGoodGoalPose() const
  {
    return last_good_goal_pose_;
  }

  /// @brief Stores a pose as the last starting pose with a good IK solution
  inline void setLastGoodStartPose(tf::Transform pose)
  {
    last_good_start_pose_ = pose;
  }

  /// @brief Stores a poase as the last goal pose with a good IK solution
  inline void setLastGoodGoalPose(tf::Transform pose)
  {
    last_good_goal_pose_ = pose;
  }

  /// @brief Sets the kinematic state corresponding to the starting joint state of the robot.
  inline void setStartState(planning_models::KinematicState* state)
  {
    start_state_ = state;
    setStateChanged(true);
  }

  /// @brief Sets the kinematic state corresponding to the goal joint state of the robot.
  inline void setGoalState(planning_models::KinematicState* state)
  {
    goal_state_ = state;
    setStateChanged(true);
  }

  /// @brief Deletes the kinematic states associated with the motion plan request.
  inline void reset()
  {
    if(start_state_ != NULL)
    {
      delete start_state_;
      start_state_ = NULL;
    }

    if(goal_state_ != NULL)
    {
      delete goal_state_;
      goal_state_ = NULL;
    }
  }

  /// @brief Returns true if an IK solution was found for this request, and false otherwise.
  inline bool hasGoodIKSolution(const PositionType& type) const
  {
    if(type == StartPosition) {
      return has_good_start_ik_solution_;
    } else {
      return has_good_goal_ik_solution_;
    }
  }

  /// @brief Set whether or not an IK solution was found for the start or end of this request.
  inline void setHasGoodIKSolution(const bool& solution, const PositionType& type)
  {
    if(type == StartPosition) {
      has_good_start_ik_solution_ = solution;
    } else {
      has_good_goal_ik_solution_ = solution;
    }
  }

  /// @brief Returns a KinematicState pointer corresponding to the starting joint state of the robot.
  inline planning_models::KinematicState* getStartState()
  {
    return start_state_;
  }

  /// @brief Returns a KinematicState pointer corresponding to the goal joint constraints of the robot.
  inline planning_models::KinematicState* getGoalState()
  {
    return goal_state_;
  }

  /// @brief Returns the name of the planning group associated
  /// with the motion plan request (usually, right or left arm)
  inline std::string getGroupName() const
  { 
    return group_name_;
  }

  /// @brief Returns the name of the link that IK is performed for.
  inline std::string getEndEffectorLink() const
  {
    return end_effector_link_;
  }

  /// @brief Sets the name of the planning group associated with the
  /// motion plan request (usually, right or left arm)
  inline void setGroupName(const std::string& name)
  {
    group_name_ = name;
  }

  /// @brief Sets the name of the link that IK is performed for.
  inline void setEndEffectorLink(const std::string& name)
  {
    end_effector_link_ = name;
  }

  /// @brief Returns the unique Id of the motion plan request
  inline const std::string& getName() const
  {
    return name_;
  }
  
  unsigned int getId() const {
    return id_;
  }

  /// @brief Sets the unique Id of the motion plan request.
  inline void setId(const unsigned int id)
  {
    id_ = id;
    name_ = getMotionPlanRequestNameFromId(id_);
  }

  /// @brief Returns true if the motion plan request's colors have changed, and false otherwise.
  inline bool shouldRefreshColors() const 
  {
    return should_refresh_colors_;
  }

  /// @brief Returns true if the refresh counter has been exhausted, and false otherwise.
  inline bool hasRefreshedColors() const
  {
    return has_refreshed_colors_;
  }

  /// @brief See hasRefreshedColors
  inline void setHasRefreshedColors(bool refresh)
  {
    has_refreshed_colors_ = refresh;

    if(refresh)
    {
      should_refresh_colors_ = false;
    }
  }

  /// @brief Tell the planning scene editor to stop publishing markers for a while
  /// so that the colors of the motion plan request can be changed.
  inline void refreshColors()
  {
    should_refresh_colors_ = true;
    has_refreshed_colors_ = false;
    refresh_timer_ = ros::Duration(0.0);
  }

  /// @brief Returns true if the starting kinematic state of the robot is being published as a set
  /// of markers or false otherwise
  inline bool isStartVisible() const
  {
    return is_start_visible_;
  }

  /// @brief Returns true if the goal kinematic state of the robot is being published as a set of markers
  /// or false otherwise
  inline bool isEndVisible() const
  {
    return is_goal_visible_;
  }

  /// @brief see isStartVisible
  inline void setStartVisible(bool visible)
  {
    is_start_visible_ = visible;
  }

  /// @brief see isEndVisible
  inline void setEndVisible(bool visible)
  {
    is_goal_visible_ = visible;
  }

  /// @brief Sets both the start and end positions to be published.
  inline void show()
  {
    setStartVisible(true);
    setEndVisible(true);
  }

  /// @brief Sets both the start and end positions to invisible.
  inline void hide()
  {
    setStartVisible(false);
    setEndVisible(false);
  }

  /// @brief Shorthand for setStartVisible(true)
  inline void showStart()
  {
    setStartVisible(true);
  }

  /// @brief Shorthand for setEndVisible(true)
  inline void showGoal()
  {
    setEndVisible(true);
  }

  /// @brief Shorthand for setStartVisible(false)
  inline void hideStart()
  {
    setStartVisible(false);
  }

  /// @brief Shorthand for setEndVisible(false)
  inline void hideGoal()
  {
    setEndVisible(false);
  }

  /// @brief Returns the color of the markers of the start position.
  inline const std_msgs::ColorRGBA& getStartColor() const
  {
    return start_color_;
  }

  /// @brief Returns the color of the markers of the goal position.
  inline const std_msgs::ColorRGBA& getGoalColor() const
  {
    return goal_color_;
  }

  /// @brief Sets the color of the markers for the start position.
  inline void setStartColor(std_msgs::ColorRGBA color)
  {
    start_color_ = color;
  }

  /// @brief Sets the color of the markers for the goal position.
  inline void setGoalColor(std_msgs::ColorRGBA color)
  {
    goal_color_ = color;
  }

  /// @brief If true, then a 6DOF control and joint controls will be visible
  /// for the start position of the robot. If false, these controls will not be shown.
  inline void setStartEditable(bool editable)
  {
    is_start_editable_ = editable;
  }

  /// @brief If true, then a 6DOF control and joint controls will be visible
  /// for the goal position of the robot. If false, these controls will not be shown.
  inline void setGoalEditable(bool editable)
  {
    is_goal_editable_ = editable;
  }

  /// @brief If true, then a 6DOF control and joint controls will be visible
  /// for the start position of the robot. If false, these controls will not be shown.
  inline bool isStartEditable() const
  {
    return is_start_editable_;
  }

  /// @brief If true, then a 6DOF control and joint controls will be visible
  /// for the goal position of the robot. If false, these controls will not be shown.
  inline bool isGoalEditable() const
  {
    return is_goal_editable_;
  }

  /// @brief Set the pipeline stage associated with this motion plan request
  inline void setSource(const std::string& source)
  {
    source_ = source;
  }

  /// @brief Returns the pipeline stage associated with this motion plan request.
  inline const std::string& getSource() const
  {
    return source_;
  }

  /// @brief Gets the underlying motion plan request message.
  inline arm_navigation_msgs::MotionPlanRequest& getMotionPlanRequest()
  {
    return motion_plan_request_;
  }

  /// @brief Sets the underlying motion plan request message.
  inline void setMotionPlanRequest(const arm_navigation_msgs::MotionPlanRequest& request)
  {
    motion_plan_request_ = request;
    setGroupName(request.group_name);
    updateStartState();
    updateGoalState();
  }

  /// @brief Sets the planning scene Id that this motion plan request is associated with.
  inline void setPlanningSceneId(const unsigned int id)
  {
    planning_scene_id_ = id;
  }
  inline unsigned int getPlanningSceneId() const
  {
    return planning_scene_id_;
  }

  /// @brief Returns the unique planning scene Id that this motion plan request is associated with.
  inline std::string getPlanningSceneName() const 
  {
    return getPlanningSceneNameFromId(planning_scene_id_);
  }

  /// @brief Returns a vector of unique trajectory Ids associated with this motion plan request.
  inline std::set<unsigned int>& getTrajectories()
  {
    return trajectories_;
  }

  unsigned int getNextTrajectoryId() const {
    if(trajectories_.empty()) return 0;
    return (*trajectories_.rbegin())+1;
  }

  void addTrajectoryId(unsigned int id) {
    trajectories_.insert(id);
  }
  
  void removeTrajectoryId(unsigned int id) {
    trajectories_.erase(id);
  }

  bool hasTrajectoryId(unsigned int id) const {
    return (trajectories_.find(id) != trajectories_.end());
  }

  bool hasPathConstraints() const {
    return has_path_constraints_;
  }
  
  void setPathConstraints(bool has) {
    has_path_constraints_ = has;
  }

  bool getConstrainRoll() const {
    return constrain_roll_;
  }

  void setConstrainRoll(bool s) {
    constrain_roll_ = s;
  }

  bool getConstrainPitch() const {
    return constrain_pitch_;
  }

  void setConstrainPitch(bool s) {
    constrain_pitch_ = s;
  }

  bool getConstrainYaw() const {
    return constrain_yaw_;
  }

  void setConstrainYaw(bool s) {
    constrain_yaw_ = s;
  }

  double getRollTolerance() const {
    return roll_tolerance_;
  }

  void setRollTolerance(double s) {
    roll_tolerance_ = s;
  }

  double getPitchTolerance() const {
    return pitch_tolerance_;
  }

  void setPitchTolerance(double s) {
    pitch_tolerance_ = s;
  }

  double getYawTolerance() const {
    return yaw_tolerance_;
  }

  void setYawTolerance(double s) {
    yaw_tolerance_ = s;
  }

  void setGoalAndPathPositionOrientationConstraints(arm_navigation_msgs::MotionPlanRequest& mpr,
                                                    planning_scene_utils::PositionType type) const;

  /// @brief Fills the member marker array with small red spheres associated with collision points.
  void updateCollisionMarkers(planning_environment::CollisionModels* cm_,
                              ros::ServiceClient* distance_state_validity_service_client_);
};


////
/// Class TrajectoryData
/// @brief Convenience class wrapping a trajectory message
/// and its meta-data
////
class TrajectoryData
{
public:

  enum MarkerType {
    VISUAL,
    COLLISION,
    PADDED
  };

protected:
  std::string name_;
  unsigned int id_;
  std::string source_;
  std::string group_name_;
  unsigned int planning_scene_id_;
  unsigned int motion_plan_request_Id_;
  trajectory_msgs::JointTrajectory trajectory_;
  trajectory_msgs::JointTrajectory trajectory_error_;
  bool is_visible_;
  MarkerType marker_type_;
  bool is_playing_;
  ros::Time playback_start_time_;
  bool collisions_visible_;
  bool state_changed_;
  std_msgs::ColorRGBA color_;
  unsigned int current_trajectory_point_;
  unsigned int trajectory_bad_point_;
  planning_models::KinematicState* current_state_;
  ros::Duration duration_;
  bool should_refresh_colors_;
  bool has_refreshed_colors_;
  visualization_msgs::MarkerArray collision_markers_;
  RenderType render_type_;
  TrajectoryRenderType trajectory_render_type_;
  ros::Duration time_to_stop_;
public:

  /// @brief This counter is exhausted when the trajectory's color has changed.
  ros::Duration refresh_timer_;

  /// @brief Corresponds to the planning, filtering, or execution outcome of the trajectory.
  arm_navigation_msgs::ArmNavigationErrorCodes trajectory_error_code_;

  TrajectoryData();
  TrajectoryData(const unsigned int& id, const std::string& source, const std::string& group_name,
                 const trajectory_msgs::JointTrajectory& trajectory);
  TrajectoryData(const unsigned int& id, const std::string& source, const std::string& group_name,
                 const trajectory_msgs::JointTrajectory& trajectory, const trajectory_msgs::JointTrajectory& trajectory_error);

  /// @brief Sets the current state of the trajectory to the current trajectory point + amount.
  /// Allows for negative values. Does not overshoot the trajectory's end or start.
  void advanceThroughTrajectory(int amount);

  /// @brief Gets the closest point on the trajectory whereby the 
  /// point.time_from_start <= time - playback_start_time_.
  void advanceToNextClosestPoint(ros::Time time);

  /// @brief Sets the joint states of the current state to those specified by the joint trajectory.
  void updateCurrentState();

  /// @brief Returns true if the current state has been recently changed, and false otherwise.
  inline bool hasStateChanged() const
  { 
    return state_changed_;
  }

  /// @brief See hasStateChanged
  inline void setStateChanged(bool changed)
  {
    state_changed_ = changed;
  }

  /// @brief Returns an array of small red spheres associated with each collision point.
  inline visualization_msgs::MarkerArray& getCollisionMarkers()
  {
    return collision_markers_;
  }

  /// @brief Returns true if the collision sphers are being published, and false otherwise.
  inline bool areCollisionsVisible() const
  {
    return collisions_visible_;
  }

  /// @brief see areCollisionVisible
  inline void setCollisionsVisible(bool shown)
  {
    collisions_visible_ = shown;
  }

  /// @brief Shorthand for setCollisionsVisible(true)
  inline void showCollisions()
  {
    setCollisionsVisible(true);
  }

  /// @brief Shorthand for setCollisionsVisible(false)
  inline void hideCollisions()
  {
    setCollisionsVisible(false);
  }

  /// @brief Returns the number of discrete points in the trajectory.
  inline size_t getTrajectorySize() const
  {
    return trajectory_.points.size();
  }

  /// @brief Returns true if the trajectory's color has changed, and false otherwise.
  inline bool shouldRefreshColors() const
  {
    return should_refresh_colors_;
  }

  /// @brief Returns ture if the refresh counter has been exhausted, and false otherwise.
  inline bool hasRefreshedColors() const
  {
    return has_refreshed_colors_;
  }

  /// @brief See hasRefreshedColors
  inline void setHasRefreshedColors(bool refresh)
  {
    has_refreshed_colors_ = refresh;

    if(refresh)
    {
      should_refresh_colors_ = false;
    }
  }

  /// @brief Sets the refresh_counter and assocated booleans so that the planning scene editor will cease
  /// publishing markers for a while, allowing the color of the markers to change.
  inline void refreshColors()
  {
    should_refresh_colors_ = true;
    has_refreshed_colors_ = false;
    refresh_timer_ = ros::Duration(0.0);
  }

  /// @brief Gets what mesh to display in RVIZ.
  inline RenderType getRenderType() const
  {
    return render_type_;
  }

  /// @brief Sets what mesh to display in RVIZ.
  inline void setRenderType(RenderType renderType)
  {
    render_type_ = renderType;
  }

  /// @brief Gets what trajectory rendering method to display in RVIZ.
  inline TrajectoryRenderType getTrajectoryRenderType() const
  {
    return trajectory_render_type_;
  }

  /// @brief Sets what trajectory rendering method to display in RVIZ.
  inline void setTrajectoryRenderType(TrajectoryRenderType renderType)
  {
    trajectory_render_type_ = renderType;
  }

  /// @brief Deletes the kinematic states associated with the trajectory.
  inline void reset()
  {

    if(current_state_ != NULL)
    {
      delete current_state_;
      current_state_ = NULL;
    }

    is_playing_ = false;
    is_visible_ = false;
    current_trajectory_point_ = 0;
    state_changed_ = false;
  }

  /// @brief Gets the current kinematic state displayed by the planning scene editor. This state is also
  /// checked for collisions.
  inline planning_models::KinematicState* getCurrentState()
  {
    return current_state_;
  }

  /// @brief see getCurrentState
  inline void setCurrentState(planning_models::KinematicState* state)
  {
    current_state_ = state;
    state_changed_ = true;
  }

  /// @brief Sets the unique Id corresponding to the motion plan request associated with this trajectory.
  inline void setMotionPlanRequestId(const unsigned int& Id)
  {
    motion_plan_request_Id_ = Id;
  }

  /// @brief See setMotionPlanRequestId
  inline const unsigned int& getMotionPlanRequestId() const
  {
    return motion_plan_request_Id_;
  }

  inline void setPlanningSceneId(const unsigned int id) {
    planning_scene_id_ = id;
  }

  unsigned int getPlanningScendId() const {
    return planning_scene_id_;
  }

  /// @brief Sets the current joint trajectory point displayed in Rviz.
  inline void setCurrentPoint(unsigned int point)
  {
    current_trajectory_point_ = point;
    state_changed_ = true;
  }

  /// @brief see setCurrentPoint
  inline unsigned int getCurrentPoint() const
  {
    return current_trajectory_point_;
  }

  /// @brief Returns the trajectory point where an error occurred.
  inline unsigned int getBadPoint() const
  {
    return trajectory_bad_point_;
  }

  /// @brief Sets the planning group name of the trajectory (usually right arm or left arm)
  inline void setGroupname(const std::string& group_name)
  {
    group_name_ = group_name;
  }

  /// @brief Returns true if the current state is automatically marching through trajectory points, and false ow.
  inline bool isPlaying() const
  {
    return is_playing_;
  }

  /// @brief Starts playback of trajectory
  inline void play()
  {
    is_playing_ = true;
    playback_start_time_ = ros::Time::now();
  }

  /// @brief Stops playback of trajectory
  inline void stop()
  {
    is_playing_ = false;
  }

  /// @brief Returns true if the current state is being shown in rviz, and false otherwise.
  inline bool isVisible() const
  {
    return is_visible_;
  }

  /// @brief See isVisible
  inline void setVisible(bool visible)
  {
    is_visible_ = visible;
  }

  inline MarkerType getMarkerType() const 
  {
    return marker_type_;
  }

  /// @brief Sets whether padded trimeshes are to be shown
  inline void setMarkerType(MarkerType mt) 
  {
    marker_type_ = mt;
  }

  /// @brief Shorthand for setVisible(true)
  inline void show()
  {
    setVisible(true);
  }

  /// @brief Shorthand for setVisible(false)
  inline void hide()
  {
    setVisible(false);
  }

  /// @brief For planners, returns the time it took to plan the trajectory, for filters, the time it took to
  /// filter it, and for robot monitors, the time it took to execute the trajectory.
  inline const ros::Duration& getDuration() const
  {
    return duration_;
  }

  /// @brief See getDuration
  inline void setDuration(const ros::Duration& duration)
  {
    duration_ = duration;
  }

  /// @brief Returns the color of the markers representing the current state being published in Rviz.
  inline const std_msgs::ColorRGBA& getColor() const
  {
    return color_;
  }

  /// @brief See getColor
  inline void setColor(const std_msgs::ColorRGBA& color)
  {
    color_ = color;
  }

  /// @brief Returns the pipeline stage of the trajectory.
  inline std::string getSource()
  {
    return source_;
  }

  /// @brief Returns the underlying trajectory message.
  inline trajectory_msgs::JointTrajectory& getTrajectory()
  {
    return trajectory_;
  }

  /// @brief See getSource
  inline void setSource(const std::string& source)
  {
    source_ = source;
  }

  /// @brief see getTrajectory
  inline void setTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
  {
    trajectory_ = trajectory;
  }

  /// @brief Returns the underlying trajectory.
  inline trajectory_msgs::JointTrajectory& getTrajectoryError()
  {
    return trajectory_error_;
  }

  /// @brief see getTrajectoryError
  inline void setTrajectoryError(const trajectory_msgs::JointTrajectory& trajectory_error)
  {
    trajectory_error_ = trajectory_error;
  }

  /// @brief Returns the unique Id of the trajectory
  inline unsigned int getId() const
  {
    return id_;
  }

  /// @brief Sets the unique Id of the trajectory
  inline void setId(const unsigned int& id)
  {
    id_ = id;
    name_ = getTrajectoryNameFromId(id_);
  }

  const std::string& getName() const {
    return name_;
  }

  /// @brief Gets the planning group associated with the trajectory (usually right arm or left arm)
  inline const std::string& getGroupName() const
  {
    return group_name_;
  }

  /// @brief Sets the point of the trajectory where an error occurred.
  inline void setBadPoint(unsigned int point)
  {
    trajectory_bad_point_ = point;
  }

  /// @brief Sets the plannign group name associated with the trajectory (usually right arm or left arm)
  inline void setGroupName(std::string name)
  {
    group_name_ = name;
  }

  /// @brief Checks the current state for collisions, and fills the collision marker array with red spheres
  /// for each collision point.
  void updateCollisionMarkers(planning_environment::CollisionModels* cm_, MotionPlanRequestData& motionPlanRequest,
                              ros::ServiceClient* distance_state_validity_service_client_);
};

////
/// Struct brief
/// @PlanningSceneParameters contains several parameters (mostly service call definitions)
/// used in the planning scene editor. These are populated by a launch file.
////
struct PlanningSceneParameters
{
  std::string left_ik_name_;
  std::string right_ik_name_;
  std::string non_coll_left_ik_name_;
  std::string non_coll_right_ik_name_;
  std::string left_interpolate_service_name_;
  std::string right_interpolate_service_name_;
  std::string planner_1_service_name_;
  std::string planner_2_service_name_;
  std::string proximity_space_service_name_;
  std::string proximity_space_validity_name_;
  std::string set_planning_scene_diff_name_;
  std::string trajectory_filter_1_service_name_;
  std::string trajectory_filter_2_service_name_;
  std::string proximity_space_planner_name_;
  std::string vis_topic_name_;
  std::string right_ik_link_;
  std::string left_ik_link_;
  std::string left_redundancy_;
  std::string right_redundancy_;
  std::string right_arm_group_;
  std::string left_arm_group_;
  std::string execute_left_trajectory_;
  std::string execute_right_trajectory_;
  std::string list_controllers_service_;
  std::string unload_controllers_service_;
  std::string load_controllers_service_;
  std::string switch_controllers_service_;
  std::string gazebo_model_name_;
  std::string robot_description_param_;
  bool use_robot_data_;
  bool sync_robot_state_with_gazebo_;
};

////
/// Class PlanningSceneEditor
/// @brief Class for creating, editing, and saving planning scenes.
////
class PlanningSceneEditor
{
public:
  /////
  /// Enum GeneratedShape
  /// @brief These kinds of shapes can be created by the editor.
  ////
  enum GeneratedShape
    {
      Box, Cylinder, Sphere
    };
protected:

  /////
  /// Enum MonitorStatus
  /// @brief PlanningSceneEditor monitors robot state while
  /// "use_robot_data_" is true. When in idle mode, the monitor
  /// is not recording robot state into a trajectory. In Executing
  /// mode, the monitor records to a trajectory. In Done mode, the
  /// final trajectory is saved to the trajectory map.
  /////
  enum MonitorStatus
    {
      idle, Executing, WaitingForStop, Done
    };

  /////
  /// Struct StateRegistry
  /// @brief convenience class for keeping track
  /// of KinematicStates. This must be done, because
  /// if not all kinematic states are deleted before
  /// SendPlanningScene() is called, the environment server
  /// is liable to hang.
  /////
  struct StateRegistry
  {
    planning_models::KinematicState* state;
    std::string source;
  };

  /////
  /// Struct SelectableObject
  /// @brief Struct containing an interactive marker
  /// for 6DOF control, and another for selection.
  ////
  struct SelectableObject
  {
    SelectableObject() {
      attach_ = false;
      detach_ = false;
      attached_collision_object_.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
      collision_object_.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
    }

    bool attach_;
    bool detach_;
    arm_navigation_msgs::AttachedCollisionObject attached_collision_object_;
    arm_navigation_msgs::CollisionObject collision_object_;
    visualization_msgs::InteractiveMarker selection_marker_;
    visualization_msgs::InteractiveMarker control_marker_;
    std_msgs::ColorRGBA color_;

    std::string id_;
  };

  /////
  /// Struct IKController
  /// @brief Struct containing the start and end 6DOF controllers
  /// for a specific motion plan request.
  /////
  struct IKController
  {
    unsigned int motion_plan_id_;
    visualization_msgs::InteractiveMarker start_controller_;
    visualization_msgs::InteractiveMarker end_controller_;
  };

  /////
  /// @brief Pure virtual function called when a trajectory,
  /// motion plan request, or the robot's state is changed.
  /////
  virtual void updateState()
  {
  }

  //////
  /// @brief Virtual function called when the planner is invoked.
  /// @param errorCode, the result of the plan.
  //////
  virtual void planCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode) = 0;

  ///////
  /// @brief Virtual function called when the filter is invoked.
  /// @param errorCode, the result of the filter call.
  //////
  virtual void filterCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode) = 0;

  virtual void attachObjectCallback(const std::string& name) = 0;
  // virtual void detachObjectCallback(arm_navigation_msgs::CollisionObject& object) = 0;

  void changeToAttached(const std::string& name);

  //////
  /// @brief Called when the selected trajectory of the selected motion plan updates its current point.
  /// Override to do something useful.
  /// @param new_current_point the new current point in the selected trajectory.
  //////
  virtual void selectedTrajectoryCurrentPointChanged( unsigned int new_current_point ) {}

  boost::recursive_mutex lock_scene_;
  arm_navigation_msgs::ArmNavigationErrorCodes last_collision_set_error_code_;
  move_arm_warehouse::MoveArmWarehouseLoggerReader* move_arm_warehouse_logger_reader_;
  planning_environment::CollisionModels* cm_;
  planning_models::KinematicState* robot_state_;
  PlanningSceneParameters params_;
  ros::NodeHandle nh_;
  ros::Publisher clock_publisher_;
  ros::Publisher joint_state_publisher_;
  ros::Publisher vis_marker_array_publisher_;
  ros::Publisher vis_marker_publisher_;
  ros::Subscriber joint_state_subscriber_;
  ros::Subscriber r_arm_controller_state_subscriber_;
  ros::Subscriber l_arm_controller_state_subscriber_;
  ros::ServiceClient collision_proximity_planner_client_;
  ros::ServiceClient distance_aware_service_client_;
  ros::ServiceClient distance_state_validity_service_client_;
  ros::ServiceClient set_planning_scene_diff_client_;
  ros::ServiceClient left_ik_service_client_;
  ros::ServiceClient left_interpolate_service_client_;
  ros::ServiceClient non_coll_left_ik_service_client_;
  ros::ServiceClient non_coll_right_ik_service_client_;
  ros::ServiceClient planning_1_service_client_;
  ros::ServiceClient planning_2_service_client_;
  ros::ServiceClient right_ik_service_client_;
  ros::ServiceClient right_interpolate_service_client_;
  ros::ServiceClient trajectory_filter_1_service_client_;
  ros::ServiceClient trajectory_filter_2_service_client_;
  ros::ServiceClient gazebo_joint_state_client_;
  ros::ServiceClient list_controllers_client_;
  ros::ServiceClient load_controllers_client_;
  ros::ServiceClient unload_controllers_client_;
  ros::ServiceClient switch_controllers_client_;
  ros::ServiceClient pause_gazebo_client_;
  ros::ServiceClient unpause_gazebo_client_;
  ros::ServiceClient set_link_properties_client_;
  ros::ServiceClient get_link_properties_client_;

  std::map<std::string, double> robot_state_joint_values_;
  std::vector<ros::Time> last_creation_time_query_;
  tf::TransformBroadcaster transform_broadcaster_;
  tf::TransformListener transform_listener_;
  std::map<std::string, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>*> arm_controller_map_;
  unsigned int max_trajectory_id_;
  unsigned int max_collision_object_id_;

  bool warehouse_data_loaded_once_;
  int  active_planner_index_;
  bool use_primary_filter_;

  trajectory_msgs::JointTrajectory logged_trajectory_;
  trajectory_msgs::JointTrajectory logged_trajectory_controller_error_;
  ros::Time logged_trajectory_start_time_;
  int logged_trajectory_controller_error_code_;

  bool send_collision_markers_;
  std::string collision_marker_state_;
  visualization_msgs::MarkerArray collision_markers_;
  planning_models::KinematicState* paused_collision_state_;
  std_msgs::ColorRGBA point_color_;
  std::vector<geometry_msgs::TransformStamped> robot_transforms_;

  interactive_markers::MenuHandler::FeedbackCallback attached_collision_object_feedback_ptr_;
  interactive_markers::MenuHandler::FeedbackCallback collision_object_selection_feedback_ptr_;
  interactive_markers::MenuHandler::FeedbackCallback collision_object_movement_feedback_ptr_;
  interactive_markers::MenuHandler::FeedbackCallback ik_control_feedback_ptr_;
  interactive_markers::MenuHandler::FeedbackCallback joint_control_feedback_ptr_;

  interactive_markers::InteractiveMarkerServer* interactive_marker_server_;

  std::map<std::string, SelectableObject>* selectable_objects_;
  std::map<std::string, IKController>* ik_controllers_;

  std::string current_planning_scene_name_;
  std::string selected_motion_plan_name_;
  std::string selected_trajectory_name_;
  std::string logged_group_name_;
  std::string logged_motion_plan_request_;
  std::map<string, MenuEntryMap> menu_entry_maps_;
  MenuHandlerMap menu_handler_map_;

  std::map<string, ros::ServiceClient*>* collision_aware_ik_services_;
  std::map<string, ros::ServiceClient*>* non_collision_aware_ik_services_;
  std::map<string, ros::ServiceClient*>* interpolated_ik_services_;
  std::map<string, arm_navigation_msgs::ArmNavigationErrorCodes> error_map_;
  std::vector<StateRegistry> states_;

  interactive_markers::MenuHandler::EntryHandle last_resize_handle_;

  std_msgs::ColorRGBA last_collision_object_color_;
  std_msgs::ColorRGBA last_mesh_object_color_;

  MonitorStatus monitor_status_;
  ros::Time time_of_controller_done_callback_;
  ros::Time time_of_last_moving_notification_;

  ros::Time last_marker_start_time_;
  ros::Duration marker_dt_;

  void attachCollisionObject(const std::string& name, 
                             const std::string& link_name,
                             const std::vector<std::string>& touch_links);
  
  /////
  /// @brief Registers a collision object as a selectable marker.
  /////
  void createSelectableMarkerFromCollisionObject(arm_navigation_msgs::CollisionObject& object, std::string name,
                                                 std::string description, std_msgs::ColorRGBA color, bool insert_selectable=true);

  
  
public:

  /// @brief Map containing all planning scenes, indexed by (unique) name.
  std::map<std::string, PlanningSceneData> planning_scene_map_;

  /// @brief Map containing all trajectories, indexed by (unique) motion plan id name, and then by unique trajectory name.
  std::map<std::string, std::map<std::string, TrajectoryData> > trajectory_map_;

  /// @brief Map containing all motion plan requests, indexed by (unique) name.
  std::map<std::string, MotionPlanRequestData> motion_plan_map_;

  /// @brief Map of joint controls and whether they have been clicked by the user.
  std::map<std::string, bool> joint_clicked_map_;

  /// @brief Map of joint controls and their last transforms.
  std::map<std::string, tf::Transform> joint_prev_transform_map_;

  PlanningSceneEditor();
  PlanningSceneEditor(PlanningSceneParameters& params);
  ~PlanningSceneEditor();

  /////
  /// @brief Calls the trajectory filter service on the given trajectory.
  /// @param requestData the request associated with the trajectory.
  /// @param trajectory the trajectory to filter.
  /// @param filter_id the id of the filtered trajectory to be returned.
  /// @return true on filtering success, or false on failure.
  /////
  bool filterTrajectory(MotionPlanRequestData& requestData, TrajectoryData& trajectory, unsigned int& filter_id);

  //////
  /// @brief loads motion plan requests associated with the given timestamp from the warehouse.
  /// @param time the time stamp associated with the planning scene.
  /// @param ids vector of strings to be filled with the ids of motion plan requests associated with that time.
  /// @param stages vector of strings to be filled with the planning stages associated with each motion plan request.
  /// @param requests vector of MotionPlanRequests to be filled with the requests associated with the given time.
  /// @return true if query to warehouse was successful, false otherwise.
  /////
  bool getAllAssociatedMotionPlanRequests(const unsigned int id, 
                                          std::vector<unsigned int>& ids,
                                          std::vector<std::string>& stages,
                                          std::vector<arm_navigation_msgs::MotionPlanRequest>& requests);

  //////
  /// @brief loads all paused states from the warehouse associated with the given time stamp.
  /// @param time the time stamp of the planning scene
  /// @param paused_times vector of time stamps corresponding to each paused time.
  /// @return true if the query to warehouse was successful, false otherwise.
  //////
  bool getAllAssociatedPausedStates(const unsigned int id, 
                                    std::vector<ros::Time>& paused_times);

  //////
  /// @brief loads all trajectory sources from the warehouse associated with the given time stamp.
  /// @param time the time stamp of the planning scene
  /// @param trajectory_sources a vector of strings to be filled with the trajectory sources (planner, filter, etc.)
  /// @return true if the query to the warehouse was successful, false otherwise.
  //////
  bool getAllAssociatedTrajectorySources(const unsigned int planning_id,
                                         const unsigned int mpr_id,
                                         std::vector<unsigned int>& trajectory_ids,
                                         std::vector<std::string>& trajectory_sources);

  /////
  /// @brief loads all planning scene times from the warehouse.
  /// @param planning_scene_times a vector of time stamps corresponding to each planning scene
  /// @return true if the query to the warehouse was successful, false otherwise.
  /////
  bool getAllPlanningSceneTimes(std::vector<ros::Time>& planning_scene_times,
                                vector<unsigned int>& planning_scene_ids);


  //////
  /// @brief loads a specific motion plan request from the warehouse.
  /// @param time the time stamp of the planning scene to load from.
  /// @param stage the planning stage associated with the request
  /// @param mpr the MotionPlanRequest message to fill with data from the warehouse.
  /// @param id the id of the request to be generated.
  /// @param planning_scene_id the planning scene id associated with the given time.
  /// @return true if the query to the warehouse was successful, false otherwise
  //////
  bool getMotionPlanRequest(const ros::Time& time, const std::string& stage,
                            arm_navigation_msgs::MotionPlanRequest& mpr, std::string& id,
                            std::string& planning_scene_id);

  /////
  /// @brief loads a specific paused state from the warehouse
  /// @param time the time stamp associated with the planning scene
  /// @param paused_time time when the paused state occurred.
  /// @param paused_state message to be filled by the warehouse.
  /// @return true if the query to teh warehouse was successful, false otherwise
  /////
  bool getPausedState(const unsigned int id, 
                      const ros::Time& paused_time,
                      head_monitor_msgs::HeadMonitorFeedback& paused_state);

  //////
  /// @brief loads all the error codes associated with a particular planning scene from the warehouse.
  /// @param time the time stamp of the planning scene
  /// @param pipeline_stages vector of strings to be filled with all request stages (planner, filter, etc.)
  /// @param error_codes vector of arm navigation error codes to be filled by the warehouse.
  /// @param error_map associates each error code with a trajectory id. To be filled by the warehouse.
  /// @return true if the query to the warehouse was successful, false otherwise.
  //////
  bool getPlanningSceneOutcomes(const unsigned int id, 
                                std::vector<std::string>& pipeline_stages,
                                std::vector<arm_navigation_msgs::ArmNavigationErrorCodes>& error_codes,
                                std::map<std::string, arm_navigation_msgs::ArmNavigationErrorCodes>& error_map);


  //////
  /// @brief loads a particular planning scene from the warehouse
  /// @param time the time stamp associated with the planning scene
  /// @param id the id of the planning scene to be filled by the function.
  /// @return true if the query to the warehouse was successful, false otherwise.
  ///////
  bool loadPlanningScene(const ros::Time& time, 
                         const unsigned int id);

  //////
  /// @brief invokes the planner service to plan from the current robot state to the given kinematic state.
  /// @param state the state to plan to.
  /// @param group_name the group to invoke the request for.
  /// @param end_effector_name the link that IK was performed on.
  /// @param constrain should the planner constrain pitch and roll of the end effector?
  /// @param trajectoryid_Out the new planned trajectory id to be filled by the function.
  /// @param planning_scene_name the id of the planning scene that this plan occurs in.
  /// @return true if the planner was successful, and false otherwise
  //////
  bool planToKinematicState(const planning_models::KinematicState& state, const std::string& group_name,
                            const std::string& end_effector_name, unsigned int& trajectoryid_Out,
                            unsigned int& planning_scene_id);

  /////
  /// @brief invokes the planner to plan from the start position of the request to the goal position.
  /// @param data the motion plan request to plan for.
  /// @param trajectoryid_Out the new planned trajectory id to be filled by the function.
  /// @return true if the planner was successful, false otherwise.
  //////
  bool planToRequest(MotionPlanRequestData& data, unsigned int& trajectoryid_Out);

  /////
  /// @brief invokes the planner to plan from the start position of the request to the goal position.
  /// @param requestid the motion plan request to plan for.
  /// @param trajectoryid_Out the new planned trajectory id to be filled by the function.
  /// @return true if the planner was successful, false otherwise.
  //////
  bool planToRequest(const std::string& requestid, unsigned int& trajectoryid_Out);

  /////
  /// @brief non-blocking call resetting the given trajectory and setting it to play and be visible.
  /// @param requestData the motion plan request associated with the trajectory
  /// @param data the trajectory to play.
  /// @return true if the trajectory can be played, false otherwise.
  /////
  bool playTrajectory(MotionPlanRequestData& requestData, TrajectoryData& data);

  /////
  /// @brief sends a planning scene diff message to the environment server, updating the global planning scene.
  /// @param data the planning scene to send.
  /// @return true if sending the diff was successful, and false otherwise.
  /////
  bool sendPlanningScene(PlanningSceneData& data);

  //////
  /// @brief invokes the inverse kinematics solver on the given motion plan requests' start or end state,
  /// setting the joint values of that state to the solution.
  /// @param mpr the motion plan request to solve IK for.
  /// @param type solve for either the start position or the goal position.
  /// @param coll_aware should the IK solution be constrained as collision-free?
  /// @param constrain_pitch_and_roll should the IK solution maintain the pitch and roll of the end effector?
  /// @param change_redundancy alters the redundant joint of the robot by the given amount.
  /// @return true if an IK solution was found, false otherwise.
  //////
  bool solveIKForEndEffectorPose(MotionPlanRequestData& mpr, PositionType type, bool coll_aware = true,
                                 double change_redundancy = 0.0);


  ///////
  /// @brief creates an interactive marker menu with the given name, associating it with a callback function.
  /// @param menu the menu handler to register the entry in.
  /// @param entryName the name of the menu entry.
  /// @param callback the function to call when this menu entry is pressed
  /// @return the menu handle of the registered entry.
  //////
  interactive_markers::MenuHandler::EntryHandle registerMenuEntry(std::string menu, std::string entryName,
                                                                  interactive_markers::MenuHandler::FeedbackCallback& callback);



  //////
  /// @brief registers a new menu entry as a sub menu of an exitsting interactive marker menu entry
  /// @param menu the menu handler maintaining the menu.
  /// @param name the name of the entry to make a sub menu for.
  /// @param subMenu the name of the sub menu entry
  /// @param callback the function to call when this menu is clicked.
  /// @return the menu handle for the registered entry.
  //////
  interactive_markers::MenuHandler::EntryHandle registerSubMenuEntry(std::string menu, std::string name,
                                                                     std::string subMenu,
                                                                     interactive_markers::MenuHandler::FeedbackCallback& callback);

  //////
  /// @brief creates an entirely new, empty planning scene.
  /// @return the newly generated id of the planning scene.
  //////
  std::string createNewPlanningScene();

  //////
  /// @brief pure virtual function that acts as a callback when a given
  /// scene was loaded. (Use case: things like loading bars)
  /// @param scene the index of the scene that was loaded
  /// @param numScenes the total number of scenes in the warehouse
  //////
  virtual void onPlanningSceneLoaded(int scene, int numScenes)
  {
  }

  //////
  /// @brief called when a collision object is moved in rviz.
  /// @param feedback the change that occurred to the collision object.
  //////
  void collisionObjectMovementCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /////
  /// @brief called when a collision object is selected in rviz.
  /// @param feedback the change that occurred to the collision object.
  /////
  void collisionObjectSelectionCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void attachedCollisionObjectInteractiveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /////
  /// @brief creates an entirely new collision object and places it in the environment.
  /// @param pose the position and orientation of the object
  /// @param shape the type of object to create (Box, Cylinder, etc.)
  /// @param scaleX the size of the object in the x direction in meters.
  /// @param scaleY the size of the object in the y direction in meters.
  /// @param scaleZ the size of the object in the z direction in meters.
  /// @param color the color of the collision object.
  /// @return the unique id of the collision object
  /////
  std::string createCollisionObject(const std::string& name, geometry_msgs::Pose pose, GeneratedShape shape, float scaleX, float scaleY,
                                    float scaleZ, std_msgs::ColorRGBA color,
                                    bool selectable);

  std::string createMeshObject(const std::string& name, 
                               geometry_msgs::Pose pose,
                               const std::string& filename,
                               const tf::Vector3& scale,
                               std_msgs::ColorRGBA color);
  //////
  /// @brief creates a 6DOF control over the end effector of either the start or goal position of the given request.
  /// @param data the motion plan request to create a 6DOF control over
  /// @param type either the start or goal position of the request
  /// @param rePose, if the interactive marker already exists, should it be re-posed?
  //////
  void createIKController(MotionPlanRequestData& data, PositionType type, bool rePose = true);

  /////
  /// @brief creates both the start and goal 6DOF controls for the given request, but only if those are visible and
  /// editable.
  /// @param data the motion plan request to create 6DOF controls for.
  /// @param rePose, if the interactive markers already exist, should they be re-posed?
  //////
  void createIkControllersFromMotionPlanRequest(MotionPlanRequestData& data, bool rePose = true);

  /////
  /// @brief creates 1DOF controls for each of the joints of the given request and its start and end positions.
  /// @param data the motion plan request to create joint controls for
  /// @param position either the start or goal position of the request.
  /////
  void createJointMarkers(MotionPlanRequestData& data, planning_scene_utils::PositionType position);


  /////
  /// @brief Creates an entirely new motion plan request with the given parameters.
  /// @param start_state the kinematic state that the robot begins in.
  /// @param end_state the kinematic state to plan to.
  /// @param group_name the group that all plans will be performed for (joints outside the group are ignored)
  /// @param end_effector_name the link that IK will be solved for.
  /// @param constrain should the request constrain the pitch and roll of the end effector?
  /// @param planning_scene_name the id of the planning scene that this request occurs in.
  /// @param motionPlan_id_Out the id of the new motion plan request.
  /// @param fromRobotState should the request start from the robot's current state, ignoring start_state?
  /////
  void createMotionPlanRequest(const planning_models::KinematicState& start_state,
                               const planning_models::KinematicState& end_state, 
                               const std::string& group_name,
                               const std::string& end_effector_name, 
                               const unsigned int& planning_scene_name,
                               const bool fromRobotState, 
                               unsigned int& motionPlan_id_Out);


  /////
  /// @brief fills the motion_plan_map with new motion plan requests initialized with warehouse data.
  /// @param planning_scene_id the id of the planning scene these requests occur in.
  /// @param ids a vector containing all the motion plan request ids
  /// @param stages a vector containing all the motion plan request stages.
  /// @param requests a vector containing all the motion plan request messages from the warehouse.
  //////
  void initMotionPlanRequestData(const unsigned int& planning_scene_id, 
                                 const std::vector<unsigned int>& ids,
                                 const std::vector<std::string>& stages,
                                 const std::vector<arm_navigation_msgs::MotionPlanRequest>& requests);


  /////
  /// @brief erases all the interactive 1DOF markers on the joints of the given request.
  /// @param data the motion plan request to erase.
  /// @param type erase either the start or goal joint controls.
  /////
  void deleteJointMarkers(MotionPlanRequestData& data, PositionType type);

  /////
  /// @brief All kinematic states in the editor are kept track of, and must be deleted with this function
  /// before the planning scene can be sent to the environment server. Otherwise, the editor will hang.
  //////
  void deleteKinematicStates();

  /////
  /// @brief erases the given motion plan request and all its associated trajectories.
  /// @param id the id of the motion plan request to delete
  /////
  void deleteMotionPlanRequest(const unsigned int& id,
                               std::vector<unsigned int>& erased_trajectories);

  /////
  /// @brief erases the given trajectory from the trajectory map.
  /// @param id the id of the trajectory to delete.
  /////
  void deleteTrajectory(unsigned int mpr_id, unsigned int traj_id);


  /////
  /// @brief Creates orientation constraints from a given robot state.
  /// @param state the state to find orientation constraints for
  /// @param end_effector_link the link whose pose should be constrained
  /// @param goal_constraint constraint filled by the function which maintains the pitch and roll of end effector.
  /// @param path_constraint constraint filled by the function which maintains the pitch and roll of end effector.
  /////
  // void determineOrientationConstraintsGivenState(const MotionPlanRequestData& mpr,
  //                                                const planning_models::KinematicState& state,
  //                                                arm_navigation_msgs::OrientationConstraint& goal_constraint,
  //                                                arm_navigation_msgs::OrientationConstraint& path_constraint);

  /////
  /// @brief if real robot data is being used, this can be used to send a trajectory to the robot for execution.
  /// @param trajectory_id the id of the trajectory to execute.
  /////
  void executeTrajectory(const std::string& mpr_name, const std::string& traj_name);

  /////
  /// @brief if real robot data is being used, this can be used to send a trajectory to the robot for execution.
  /// @param data the trajectory to execute.
  /////
  void executeTrajectory(TrajectoryData& data);

  //////
  /// @brief gets TF data for the given kinematic state.
  /// @param state the kinematic state to produce transforms for
  /// @param trans_vector the vector of TF transforms to be filled by the function.
  /// @param stamp the time stamp to apply to each transform.
  //////
  void getAllRobotStampedTransforms(const planning_models::KinematicState& state,
                                    vector<geometry_msgs::TransformStamped>& trans_vector, const ros::Time& stamp);

  //////
  /// @brief Fills the given array with mesh markers associated with all motion plan requests.
  /// @param arr the marker array to fill with mesh markers.
  //////
  void getMotionPlanningMarkers(visualization_msgs::MarkerArray& arr);

  /////
  /// @brief Fills the given array with mesh markers associated with all trajectories.
  /// @param arr the marker array to fill with mesh markers.
  /////
  void getTrajectoryMarkers(visualization_msgs::MarkerArray& arr);

  //////
  /// @brief Called when a 6DOF interactive marker is altered in RVIZ.
  /// @param feedback the change that occurred to the marker.
  //////
  void IKControllerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  //////
  /// @brief Called when a 1DOF joint marker is altered in RVIZ.
  /// @param feedback the change that occured to the marker.
  //////
  void JointControllerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /////
  /// @brief Called when the robot monitor detects that the robot has stopped following a trajectory.
  /// @param state the goal of the trajectory.
  /// @param result what the controller did while executing the trajectory.
  /////
  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const control_msgs::FollowJointTrajectoryResultConstPtr& result);

  /////
  /// @brief Called when the robot actually stops moving, following execution of the trajectory.
  /////
  void armHasStoppedMoving();

  /////
  /// @brief Called when the robot monitor detects a change in the robot state.
  /// @param joint_state the new state of the robot.
  /////
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);

/////
/// @brief Called when the arm controller has a state update
/// @param joint control state of the arm controller.
/////
void jointTrajectoryControllerStateCallback(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr& joint_state);


  /////
  /// @brief gets all the motion plan requests, trajectories, and planning scenes from the warehouse.
  /////
  void loadAllWarehouseData();

  /////
  /// @brief creates an interactive marker for rotation joint control.
  /// @param transform the position and orientation of the marker.
  /// @param axis the axis of rotation
  /// @param name the id of the marker
  /// @param desciption the text to be displayed above the marker
  /// @param scale the size of the marker's radius, in meters.
  /// @param angle the initial angle of the marker about its axis.
  /////
  void makeInteractive1DOFRotationMarker(tf::Transform transform, tf::Vector3 axis, string name, string description,
                                         float scale = 1.0f, float angle = 0.0f);

  //////
  /// @brief creates an interactive marker for prismatic joint control.
  /// @param transform the position and orientation of the marker.
  /// @param axis the axis of translation of the prismatic joint.
  /// @param name the id of the marker.
  /// @param description the text to display above the marker.
  /// @param scale the size of the marker in meters.
  /// @param value the initial translation of the prismatic joint along its axis.
  //////
  void makeInteractive1DOFTranslationMarker(tf::Transform transform, tf::Vector3 axis, string name, string description,
                                            float scale = 1.0f, float value = 0.0f);

  //////
  /// @brief for debugging, prints the given trajectory point values.
  //////
  void printTrajectoryPoint(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values);

  /////
  /// @brief sets the motion plan request start or goal to a set of random joint values. Avoids collisions
  /// @param mpr the motion plan request to randomize.
  /// @param type either the start or goal of the motion plan request.
  /////
  void randomlyPerturb(MotionPlanRequestData& mpr, PositionType type);

  //////
  /// @brief Pushes the given planning scene to the warehouse with ros::WallTime::now() as its timestamp.
  /// @param data the planning scene to push to the warehouse.
  //////
  void savePlanningScene(PlanningSceneData& data, bool copy = false);

  /////
  /// @brief sends all stored mesh and sphere markers for collisions and links to rviz.
  /////
  void sendMarkers();

  /////
  /// @brief sends all TF transforms and a wall clock time to ROS.
  /////
  void sendTransformsAndClock();

  void makeSelectableAttachedObjectFromPlanningScene(const arm_navigation_msgs::PlanningScene& scene,
                                                     arm_navigation_msgs::AttachedCollisionObject& att);

  //////
  /// @brief loads the given planning scene from the warehouse.
  /// @param id the id of the planning scene to load.
  /// @param loadRequests should the motion plan requests be loaded as well?
  /// @param loadTrajectories should the trajectories be loaded as well?
  //////
  void setCurrentPlanningScene(std::string id, bool loadRequests = true, bool loadTrajectories = true);

  /////
  /// @brief either shows or hides the 6DOF interactive markers associated with the given request.
  /// @param id the id of the motion plan request.
  /// @param type either the start or goal position of the request.
  /// @param visible should the 6DOF controller be shown or not?
  /////
  void setIKControlsVisible(std::string id, PositionType type, bool visible);

  /////
  /// @brief attempts to set the state of the given joint in the given motion plan request so that it matches the
  /// given transform.
  /// @param data the motion plan request to set joint states for.
  /// @param position either the start or goal position of the motion plan request.
  /// @param jointName the joint to set the state for.
  /// @param value the joint will attempt to match this position and orientation.
  /////
  void setJointState(MotionPlanRequestData& data, PositionType position, std::string& jointName, tf::Transform value);

  /////
  /// @brief if robot data is not being used, publishes fake joint states of the current robot state to
  /// a robot state publisher node.
  /////
  void updateJointStates();

  bool hasTrajectory(const std::string& mpr_name,
                     const std::string& traj_name) const;

  /////
  /// @brief generates a unique collision object id.
  /// @return the newly generated id.
  /////
  inline std::string generateNewCollisionObjectId()
  {
    std::stringstream stream;
    stream << "collision_object_";
    max_collision_object_id_++;
    stream << max_collision_object_id_;
    return stream.str();
  }

  /////
  /// @brief generates a unique planning scene id.
  /// @return the newly generated id.
  /////
  inline unsigned int generateNewPlanningSceneId()
  {
    return move_arm_warehouse_logger_reader_->determineNextPlanningSceneId();
  }


  //////
  /// @brief Removes the collision object with the specified name from the world.
  /// @param name the unique id of the object.
  //////
  void deleteCollisionObject(std::string& name);


  inline void lockScene()
  {
    lock_scene_.lock();
  }

  inline void unlockScene()
  {
    lock_scene_.unlock();
  }

  inline planning_environment::CollisionModels* getCollisionModel()
  {
    return cm_;
  }

  inline void setCollisionModel(planning_environment::CollisionModels* model, bool shouldDelete = false)
  {
    if(shouldDelete && cm_ != NULL)
    {
      delete cm_;
      cm_ = model;
    }
    cm_ = model;
  }

  inline planning_models::KinematicState* getRobotState()
  {
    return robot_state_;
  }

  inline void setRobotState(planning_models::KinematicState* robot_state, bool shouldDelete = true)
  {
    if(shouldDelete && robot_state_ != NULL)
    {
      delete robot_state_;
      robot_state_ = NULL;
    }

    robot_state_ = robot_state;
  }

  inline move_arm_warehouse::MoveArmWarehouseLoggerReader* getLoggerReader()
  {
    return move_arm_warehouse_logger_reader_;
  }

  inline void setLoggerReader(move_arm_warehouse::MoveArmWarehouseLoggerReader* loggerReader,
                              bool shouldDelete = true)
  {
    if(move_arm_warehouse_logger_reader_ != NULL)
    {
      delete move_arm_warehouse_logger_reader_;
      move_arm_warehouse_logger_reader_ = NULL;
    }

    move_arm_warehouse_logger_reader_ = loggerReader;
  }
};

}
#endif
