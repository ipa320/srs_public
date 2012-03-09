FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/srs_grasping/msg"
  "../src/srs_grasping/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/GraspCAction.msg"
  "../msg/GraspCGoal.msg"
  "../msg/GraspCActionGoal.msg"
  "../msg/GraspCResult.msg"
  "../msg/GraspCActionResult.msg"
  "../msg/GraspCFeedback.msg"
  "../msg/GraspCActionFeedback.msg"
  "../msg/GraspFAction.msg"
  "../msg/GraspFGoal.msg"
  "../msg/GraspFActionGoal.msg"
  "../msg/GraspFResult.msg"
  "../msg/GraspFActionResult.msg"
  "../msg/GraspFFeedback.msg"
  "../msg/GraspFActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
