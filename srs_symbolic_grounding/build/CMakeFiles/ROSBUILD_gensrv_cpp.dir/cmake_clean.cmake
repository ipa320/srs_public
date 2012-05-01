FILE(REMOVE_RECURSE
  "../src/srs_symbolic_grounding/msg"
  "../src/srs_symbolic_grounding/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/srs_symbolic_grounding/SymbolGroundingExploreBasePose.h"
  "../srv_gen/cpp/include/srs_symbolic_grounding/SymbolGroundingGraspBasePose.h"
  "../srv_gen/cpp/include/srs_symbolic_grounding/SymbolGroundingPrimitiveBasePose.h"
  "../srv_gen/cpp/include/srs_symbolic_grounding/SymbolGroundingGraspBasePoseExperimental.h"
  "../srv_gen/cpp/include/srs_symbolic_grounding/GetWorkspaceOnMap.h"
  "../srv_gen/cpp/include/srs_symbolic_grounding/SymbolGroundingScanBasePose.h"
  "../srv_gen/cpp/include/srs_symbolic_grounding/GetRobotBasePose.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
