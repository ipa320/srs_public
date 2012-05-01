; Auto-generated. Do not edit!


(cl:in-package srs_symbolic_grounding-srv)


;//! \htmlinclude SymbolGroundingExploreBasePose-request.msg.html

(cl:defclass <SymbolGroundingExploreBasePose-request> (roslisp-msg-protocol:ros-message)
  ((parent_obj_geometry
    :reader parent_obj_geometry
    :initarg :parent_obj_geometry
    :type srs_symbolic_grounding-msg:SRSFurnitureGeometry
    :initform (cl:make-instance 'srs_symbolic_grounding-msg:SRSFurnitureGeometry))
   (furniture_geometry_list
    :reader furniture_geometry_list
    :initarg :furniture_geometry_list
    :type (cl:vector srs_symbolic_grounding-msg:SRSFurnitureGeometry)
   :initform (cl:make-array 0 :element-type 'srs_symbolic_grounding-msg:SRSFurnitureGeometry :initial-element (cl:make-instance 'srs_symbolic_grounding-msg:SRSFurnitureGeometry))))
)

(cl:defclass SymbolGroundingExploreBasePose-request (<SymbolGroundingExploreBasePose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SymbolGroundingExploreBasePose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SymbolGroundingExploreBasePose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_symbolic_grounding-srv:<SymbolGroundingExploreBasePose-request> is deprecated: use srs_symbolic_grounding-srv:SymbolGroundingExploreBasePose-request instead.")))

(cl:ensure-generic-function 'parent_obj_geometry-val :lambda-list '(m))
(cl:defmethod parent_obj_geometry-val ((m <SymbolGroundingExploreBasePose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:parent_obj_geometry-val is deprecated.  Use srs_symbolic_grounding-srv:parent_obj_geometry instead.")
  (parent_obj_geometry m))

(cl:ensure-generic-function 'furniture_geometry_list-val :lambda-list '(m))
(cl:defmethod furniture_geometry_list-val ((m <SymbolGroundingExploreBasePose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:furniture_geometry_list-val is deprecated.  Use srs_symbolic_grounding-srv:furniture_geometry_list instead.")
  (furniture_geometry_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SymbolGroundingExploreBasePose-request>) ostream)
  "Serializes a message object of type '<SymbolGroundingExploreBasePose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'parent_obj_geometry) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'furniture_geometry_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'furniture_geometry_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SymbolGroundingExploreBasePose-request>) istream)
  "Deserializes a message object of type '<SymbolGroundingExploreBasePose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'parent_obj_geometry) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'furniture_geometry_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'furniture_geometry_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'srs_symbolic_grounding-msg:SRSFurnitureGeometry))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SymbolGroundingExploreBasePose-request>)))
  "Returns string type for a service object of type '<SymbolGroundingExploreBasePose-request>"
  "srs_symbolic_grounding/SymbolGroundingExploreBasePoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SymbolGroundingExploreBasePose-request)))
  "Returns string type for a service object of type 'SymbolGroundingExploreBasePose-request"
  "srs_symbolic_grounding/SymbolGroundingExploreBasePoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SymbolGroundingExploreBasePose-request>)))
  "Returns md5sum for a message object of type '<SymbolGroundingExploreBasePose-request>"
  "159a1f856b1040c3078859c07478c7c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SymbolGroundingExploreBasePose-request)))
  "Returns md5sum for a message object of type 'SymbolGroundingExploreBasePose-request"
  "159a1f856b1040c3078859c07478c7c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SymbolGroundingExploreBasePose-request>)))
  "Returns full string definition for message of type '<SymbolGroundingExploreBasePose-request>"
  (cl:format cl:nil "SRSFurnitureGeometry parent_obj_geometry~%SRSFurnitureGeometry[] furniture_geometry_list~%~%================================================================================~%MSG: srs_symbolic_grounding/SRSFurnitureGeometry~%# Point point~%# Orientation angles~%float32 l~%float32 w~%float32 h~%~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SymbolGroundingExploreBasePose-request)))
  "Returns full string definition for message of type 'SymbolGroundingExploreBasePose-request"
  (cl:format cl:nil "SRSFurnitureGeometry parent_obj_geometry~%SRSFurnitureGeometry[] furniture_geometry_list~%~%================================================================================~%MSG: srs_symbolic_grounding/SRSFurnitureGeometry~%# Point point~%# Orientation angles~%float32 l~%float32 w~%float32 h~%~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SymbolGroundingExploreBasePose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'parent_obj_geometry))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'furniture_geometry_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SymbolGroundingExploreBasePose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SymbolGroundingExploreBasePose-request
    (cl:cons ':parent_obj_geometry (parent_obj_geometry msg))
    (cl:cons ':furniture_geometry_list (furniture_geometry_list msg))
))
;//! \htmlinclude SymbolGroundingExploreBasePose-response.msg.html

(cl:defclass <SymbolGroundingExploreBasePose-response> (roslisp-msg-protocol:ros-message)
  ((explore_base_pose_list
    :reader explore_base_pose_list
    :initarg :explore_base_pose_list
    :type (cl:vector geometry_msgs-msg:Pose2D)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose2D :initial-element (cl:make-instance 'geometry_msgs-msg:Pose2D))))
)

(cl:defclass SymbolGroundingExploreBasePose-response (<SymbolGroundingExploreBasePose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SymbolGroundingExploreBasePose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SymbolGroundingExploreBasePose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_symbolic_grounding-srv:<SymbolGroundingExploreBasePose-response> is deprecated: use srs_symbolic_grounding-srv:SymbolGroundingExploreBasePose-response instead.")))

(cl:ensure-generic-function 'explore_base_pose_list-val :lambda-list '(m))
(cl:defmethod explore_base_pose_list-val ((m <SymbolGroundingExploreBasePose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:explore_base_pose_list-val is deprecated.  Use srs_symbolic_grounding-srv:explore_base_pose_list instead.")
  (explore_base_pose_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SymbolGroundingExploreBasePose-response>) ostream)
  "Serializes a message object of type '<SymbolGroundingExploreBasePose-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'explore_base_pose_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'explore_base_pose_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SymbolGroundingExploreBasePose-response>) istream)
  "Deserializes a message object of type '<SymbolGroundingExploreBasePose-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'explore_base_pose_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'explore_base_pose_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SymbolGroundingExploreBasePose-response>)))
  "Returns string type for a service object of type '<SymbolGroundingExploreBasePose-response>"
  "srs_symbolic_grounding/SymbolGroundingExploreBasePoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SymbolGroundingExploreBasePose-response)))
  "Returns string type for a service object of type 'SymbolGroundingExploreBasePose-response"
  "srs_symbolic_grounding/SymbolGroundingExploreBasePoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SymbolGroundingExploreBasePose-response>)))
  "Returns md5sum for a message object of type '<SymbolGroundingExploreBasePose-response>"
  "159a1f856b1040c3078859c07478c7c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SymbolGroundingExploreBasePose-response)))
  "Returns md5sum for a message object of type 'SymbolGroundingExploreBasePose-response"
  "159a1f856b1040c3078859c07478c7c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SymbolGroundingExploreBasePose-response>)))
  "Returns full string definition for message of type '<SymbolGroundingExploreBasePose-response>"
  (cl:format cl:nil "geometry_msgs/Pose2D[] explore_base_pose_list~%~%~%~%~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SymbolGroundingExploreBasePose-response)))
  "Returns full string definition for message of type 'SymbolGroundingExploreBasePose-response"
  (cl:format cl:nil "geometry_msgs/Pose2D[] explore_base_pose_list~%~%~%~%~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SymbolGroundingExploreBasePose-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'explore_base_pose_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SymbolGroundingExploreBasePose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SymbolGroundingExploreBasePose-response
    (cl:cons ':explore_base_pose_list (explore_base_pose_list msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SymbolGroundingExploreBasePose)))
  'SymbolGroundingExploreBasePose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SymbolGroundingExploreBasePose)))
  'SymbolGroundingExploreBasePose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SymbolGroundingExploreBasePose)))
  "Returns string type for a service object of type '<SymbolGroundingExploreBasePose>"
  "srs_symbolic_grounding/SymbolGroundingExploreBasePose")