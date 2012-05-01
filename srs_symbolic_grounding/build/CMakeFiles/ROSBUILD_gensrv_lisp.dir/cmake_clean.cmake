FILE(REMOVE_RECURSE
  "../src/srs_symbolic_grounding/msg"
  "../src/srs_symbolic_grounding/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/SymbolGroundingExploreBasePose.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SymbolGroundingExploreBasePose.lisp"
  "../srv_gen/lisp/SymbolGroundingGraspBasePose.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SymbolGroundingGraspBasePose.lisp"
  "../srv_gen/lisp/SymbolGroundingPrimitiveBasePose.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SymbolGroundingPrimitiveBasePose.lisp"
  "../srv_gen/lisp/SymbolGroundingGraspBasePoseExperimental.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SymbolGroundingGraspBasePoseExperimental.lisp"
  "../srv_gen/lisp/GetWorkspaceOnMap.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_GetWorkspaceOnMap.lisp"
  "../srv_gen/lisp/SymbolGroundingScanBasePose.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SymbolGroundingScanBasePose.lisp"
  "../srv_gen/lisp/GetRobotBasePose.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_GetRobotBasePose.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
