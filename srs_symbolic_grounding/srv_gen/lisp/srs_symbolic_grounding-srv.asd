
(cl:in-package :asdf)

(defsystem "srs_symbolic_grounding-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :srs_symbolic_grounding-msg
)
  :components ((:file "_package")
    (:file "SymbolGroundingExploreBasePose" :depends-on ("_package_SymbolGroundingExploreBasePose"))
    (:file "_package_SymbolGroundingExploreBasePose" :depends-on ("_package"))
    (:file "SymbolGroundingGraspBasePose" :depends-on ("_package_SymbolGroundingGraspBasePose"))
    (:file "_package_SymbolGroundingGraspBasePose" :depends-on ("_package"))
    (:file "SymbolGroundingPrimitiveBasePose" :depends-on ("_package_SymbolGroundingPrimitiveBasePose"))
    (:file "_package_SymbolGroundingPrimitiveBasePose" :depends-on ("_package"))
    (:file "SymbolGroundingGraspBasePoseExperimental" :depends-on ("_package_SymbolGroundingGraspBasePoseExperimental"))
    (:file "_package_SymbolGroundingGraspBasePoseExperimental" :depends-on ("_package"))
    (:file "GetWorkspaceOnMap" :depends-on ("_package_GetWorkspaceOnMap"))
    (:file "_package_GetWorkspaceOnMap" :depends-on ("_package"))
    (:file "SymbolGroundingScanBasePose" :depends-on ("_package_SymbolGroundingScanBasePose"))
    (:file "_package_SymbolGroundingScanBasePose" :depends-on ("_package"))
    (:file "GetRobotBasePose" :depends-on ("_package_GetRobotBasePose"))
    (:file "_package_GetRobotBasePose" :depends-on ("_package"))
  ))