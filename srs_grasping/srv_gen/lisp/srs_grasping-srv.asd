
(cl:in-package :asdf)

(defsystem "srs_grasping-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :srs_msgs-msg
)
  :components ((:file "_package")
    (:file "GetGraspsFromPosition" :depends-on ("_package_GetGraspsFromPosition"))
    (:file "_package_GetGraspsFromPosition" :depends-on ("_package"))
    (:file "GetGraspConfigurations" :depends-on ("_package_GetGraspConfigurations"))
    (:file "_package_GetGraspConfigurations" :depends-on ("_package"))
  ))