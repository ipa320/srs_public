FILE(REMOVE_RECURSE
  "../src/srs_symbolic_grounding/msg"
  "../src/srs_symbolic_grounding/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/srs_symbolic_grounding/srv/__init__.py"
  "../src/srs_symbolic_grounding/srv/_SymbolGroundingExploreBasePose.py"
  "../src/srs_symbolic_grounding/srv/_SymbolGroundingGraspBasePose.py"
  "../src/srs_symbolic_grounding/srv/_SymbolGroundingPrimitiveBasePose.py"
  "../src/srs_symbolic_grounding/srv/_SymbolGroundingGraspBasePoseExperimental.py"
  "../src/srs_symbolic_grounding/srv/_GetWorkspaceOnMap.py"
  "../src/srs_symbolic_grounding/srv/_SymbolGroundingScanBasePose.py"
  "../src/srs_symbolic_grounding/srv/_GetRobotBasePose.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
