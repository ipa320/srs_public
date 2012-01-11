
(cl:in-package :asdf)

(defsystem "srs_grasping-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :geometry_msgs-msg
               :srs_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "IKResult" :depends-on ("_package_IKResult"))
    (:file "_package_IKResult" :depends-on ("_package"))
    (:file "IKActionResult" :depends-on ("_package_IKActionResult"))
    (:file "_package_IKActionResult" :depends-on ("_package"))
    (:file "IKFeedback" :depends-on ("_package_IKFeedback"))
    (:file "_package_IKFeedback" :depends-on ("_package"))
    (:file "GraspFeedback" :depends-on ("_package_GraspFeedback"))
    (:file "_package_GraspFeedback" :depends-on ("_package"))
    (:file "GraspGoal" :depends-on ("_package_GraspGoal"))
    (:file "_package_GraspGoal" :depends-on ("_package"))
    (:file "IKActionFeedback" :depends-on ("_package_IKActionFeedback"))
    (:file "_package_IKActionFeedback" :depends-on ("_package"))
    (:file "IKAction" :depends-on ("_package_IKAction"))
    (:file "_package_IKAction" :depends-on ("_package"))
    (:file "GraspActionFeedback" :depends-on ("_package_GraspActionFeedback"))
    (:file "_package_GraspActionFeedback" :depends-on ("_package"))
    (:file "IKGoal" :depends-on ("_package_IKGoal"))
    (:file "_package_IKGoal" :depends-on ("_package"))
    (:file "GraspActionResult" :depends-on ("_package_GraspActionResult"))
    (:file "_package_GraspActionResult" :depends-on ("_package"))
    (:file "IKActionGoal" :depends-on ("_package_IKActionGoal"))
    (:file "_package_IKActionGoal" :depends-on ("_package"))
    (:file "GraspResult" :depends-on ("_package_GraspResult"))
    (:file "_package_GraspResult" :depends-on ("_package"))
    (:file "GraspActionGoal" :depends-on ("_package_GraspActionGoal"))
    (:file "_package_GraspActionGoal" :depends-on ("_package"))
    (:file "GraspAction" :depends-on ("_package_GraspAction"))
    (:file "_package_GraspAction" :depends-on ("_package"))
  ))