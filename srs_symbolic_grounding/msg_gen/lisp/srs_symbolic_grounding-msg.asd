
(cl:in-package :asdf)

(defsystem "srs_symbolic_grounding-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "SRSFurnitureGeometry" :depends-on ("_package_SRSFurnitureGeometry"))
    (:file "_package_SRSFurnitureGeometry" :depends-on ("_package"))
    (:file "FurnitureGeometry" :depends-on ("_package_FurnitureGeometry"))
    (:file "_package_FurnitureGeometry" :depends-on ("_package"))
    (:file "SRSSpatialInfo" :depends-on ("_package_SRSSpatialInfo"))
    (:file "_package_SRSSpatialInfo" :depends-on ("_package"))
  ))