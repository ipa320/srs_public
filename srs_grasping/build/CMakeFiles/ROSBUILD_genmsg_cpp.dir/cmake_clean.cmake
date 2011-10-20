FILE(REMOVE_RECURSE
  "../src/srs_grasping/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/srs_grasping/GraspAction.h"
  "../msg_gen/cpp/include/srs_grasping/GraspGoal.h"
  "../msg_gen/cpp/include/srs_grasping/GraspActionGoal.h"
  "../msg_gen/cpp/include/srs_grasping/GraspResult.h"
  "../msg_gen/cpp/include/srs_grasping/GraspActionResult.h"
  "../msg_gen/cpp/include/srs_grasping/GraspFeedback.h"
  "../msg_gen/cpp/include/srs_grasping/GraspActionFeedback.h"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
