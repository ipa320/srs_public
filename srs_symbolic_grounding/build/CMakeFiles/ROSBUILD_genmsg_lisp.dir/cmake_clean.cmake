FILE(REMOVE_RECURSE
  "../src/srs_symbolic_grounding/msg"
  "../src/srs_symbolic_grounding/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/SRSFurnitureGeometry.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_SRSFurnitureGeometry.lisp"
  "../msg_gen/lisp/FurnitureGeometry.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_FurnitureGeometry.lisp"
  "../msg_gen/lisp/SRSSpatialInfo.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_SRSSpatialInfo.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
