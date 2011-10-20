
(cl:in-package :asdf)

(defsystem "srs_grasping-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :srs_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GraspFeedback" :depends-on ("_package_GraspFeedback"))
    (:file "_package_GraspFeedback" :depends-on ("_package"))
    (:file "GraspGoal" :depends-on ("_package_GraspGoal"))
    (:file "_package_GraspGoal" :depends-on ("_package"))
    (:file "GraspActionFeedback" :depends-on ("_package_GraspActionFeedback"))
    (:file "_package_GraspActionFeedback" :depends-on ("_package"))
    (:file "GraspActionResult" :depends-on ("_package_GraspActionResult"))
    (:file "_package_GraspActionResult" :depends-on ("_package"))
    (:file "GraspResult" :depends-on ("_package_GraspResult"))
    (:file "_package_GraspResult" :depends-on ("_package"))
    (:file "GraspActionGoal" :depends-on ("_package_GraspActionGoal"))
    (:file "_package_GraspActionGoal" :depends-on ("_package"))
    (:file "GraspAction" :depends-on ("_package_GraspAction"))
    (:file "_package_GraspAction" :depends-on ("_package"))
  ))