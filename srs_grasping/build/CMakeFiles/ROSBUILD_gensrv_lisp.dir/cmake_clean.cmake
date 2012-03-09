FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/srs_grasping/msg"
  "../src/srs_grasping/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/GetGraspsFromPosition.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_GetGraspsFromPosition.lisp"
  "../srv_gen/lisp/GetGraspConfigurations.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_GetGraspConfigurations.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
