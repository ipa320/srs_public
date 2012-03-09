FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/srs_grasping/msg"
  "../src/srs_grasping/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/srs_grasping/GetGraspsFromPosition.h"
  "../srv_gen/cpp/include/srs_grasping/GetGraspConfigurations.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
