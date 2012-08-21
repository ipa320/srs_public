
(cl:in-package :asdf)

(defsystem "srs_symbolic_grounding-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :srs_msgs-msg
)
  :components ((:file "_package")
    (:file "SymbolGroundingScanBaseRegion" :depends-on ("_package_SymbolGroundingScanBaseRegion"))
    (:file "_package_SymbolGroundingScanBaseRegion" :depends-on ("_package"))
    (:file "SymbolGroundingGraspBasePoseExperimental" :depends-on ("_package_SymbolGroundingGraspBasePoseExperimental"))
    (:file "_package_SymbolGroundingGraspBasePoseExperimental" :depends-on ("_package"))
    (:file "SymbolGroundingGraspBaseRegion" :depends-on ("_package_SymbolGroundingGraspBaseRegion"))
    (:file "_package_SymbolGroundingGraspBaseRegion" :depends-on ("_package"))
    (:file "SymbolGroundingGraspBasePose" :depends-on ("_package_SymbolGroundingGraspBasePose"))
    (:file "_package_SymbolGroundingGraspBasePose" :depends-on ("_package"))
    (:file "SymbolGroundingExploreBasePose" :depends-on ("_package_SymbolGroundingExploreBasePose"))
    (:file "_package_SymbolGroundingExploreBasePose" :depends-on ("_package"))
    (:file "SymbolGroundingScanBasePose" :depends-on ("_package_SymbolGroundingScanBasePose"))
    (:file "_package_SymbolGroundingScanBasePose" :depends-on ("_package"))
    (:file "ScanBasePose" :depends-on ("_package_ScanBasePose"))
    (:file "_package_ScanBasePose" :depends-on ("_package"))
    (:file "SymbolGroundingDeliverBaseRegion" :depends-on ("_package_SymbolGroundingDeliverBaseRegion"))
    (:file "_package_SymbolGroundingDeliverBaseRegion" :depends-on ("_package"))
  ))