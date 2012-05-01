FILE(REMOVE_RECURSE
  "../src/srs_symbolic_grounding/msg"
  "../src/srs_symbolic_grounding/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/srs_symbolic_grounding/msg/__init__.py"
  "../src/srs_symbolic_grounding/msg/_SRSFurnitureGeometry.py"
  "../src/srs_symbolic_grounding/msg/_FurnitureGeometry.py"
  "../src/srs_symbolic_grounding/msg/_SRSSpatialInfo.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
