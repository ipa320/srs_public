FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/srs_grasping/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/IKAction.msg"
  "../msg/IKGoal.msg"
  "../msg/IKActionGoal.msg"
  "../msg/IKResult.msg"
  "../msg/IKActionResult.msg"
  "../msg/IKFeedback.msg"
  "../msg/IKActionFeedback.msg"
  "../msg/GraspAction.msg"
  "../msg/GraspGoal.msg"
  "../msg/GraspActionGoal.msg"
  "../msg/GraspResult.msg"
  "../msg/GraspActionResult.msg"
  "../msg/GraspFeedback.msg"
  "../msg/GraspActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
