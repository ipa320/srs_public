
(cl:in-package :asdf)

(defsystem "srs_symbolic_grounding-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "FurnitureGeometry" :depends-on ("_package_FurnitureGeometry"))
    (:file "_package_FurnitureGeometry" :depends-on ("_package"))
  ))