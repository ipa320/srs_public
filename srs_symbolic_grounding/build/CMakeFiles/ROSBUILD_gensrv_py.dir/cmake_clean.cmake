FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/srs_symbolic_grounding/msg"
  "../src/srs_symbolic_grounding/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/srs_symbolic_grounding/srv/__init__.py"
  "../src/srs_symbolic_grounding/srv/_SymbolGroundingScanBaseRegion.py"
  "../src/srs_symbolic_grounding/srv/_SymbolGroundingGraspBasePoseExperimental.py"
  "../src/srs_symbolic_grounding/srv/_SymbolGroundingGraspBaseRegion.py"
  "../src/srs_symbolic_grounding/srv/_SymbolGroundingGraspBasePose.py"
  "../src/srs_symbolic_grounding/srv/_SymbolGroundingExploreBasePose.py"
  "../src/srs_symbolic_grounding/srv/_SymbolGroundingScanBasePose.py"
  "../src/srs_symbolic_grounding/srv/_ScanBasePose.py"
  "../src/srs_symbolic_grounding/srv/_SymbolGroundingDeliverBaseRegion.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
