FILE(REMOVE_RECURSE
  "../src/srs_grasping/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/srs_grasping/msg/__init__.py"
  "../src/srs_grasping/msg/_GraspAction.py"
  "../src/srs_grasping/msg/_GraspGoal.py"
  "../src/srs_grasping/msg/_GraspActionGoal.py"
  "../src/srs_grasping/msg/_GraspResult.py"
  "../src/srs_grasping/msg/_GraspActionResult.py"
  "../src/srs_grasping/msg/_GraspFeedback.py"
  "../src/srs_grasping/msg/_GraspActionFeedback.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
