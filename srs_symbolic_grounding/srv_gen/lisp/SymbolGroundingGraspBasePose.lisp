; Auto-generated. Do not edit!


(cl:in-package srs_symbolic_grounding-srv)


;//! \htmlinclude SymbolGroundingGraspBasePose-request.msg.html

(cl:defclass <SymbolGroundingGraspBasePose-request> (roslisp-msg-protocol:ros-message)
  ((target_obj_pose
    :reader target_obj_pose
    :initarg :target_obj_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (parent_obj_geometry
    :reader parent_obj_geometry
    :initarg :parent_obj_geometry
    :type srs_symbolic_grounding-msg:SRSSpatialInfo
    :initform (cl:make-instance 'srs_symbolic_grounding-msg:SRSSpatialInfo))
   (furniture_geometry_list
    :reader furniture_geometry_list
    :initarg :furniture_geometry_list
    :type (cl:vector srs_symbolic_grounding-msg:SRSSpatialInfo)
   :initform (cl:make-array 0 :element-type 'srs_symbolic_grounding-msg:SRSSpatialInfo :initial-element (cl:make-instance 'srs_symbolic_grounding-msg:SRSSpatialInfo))))
)

(cl:defclass SymbolGroundingGraspBasePose-request (<SymbolGroundingGraspBasePose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SymbolGroundingGraspBasePose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SymbolGroundingGraspBasePose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_symbolic_grounding-srv:<SymbolGroundingGraspBasePose-request> is deprecated: use srs_symbolic_grounding-srv:SymbolGroundingGraspBasePose-request instead.")))

(cl:ensure-generic-function 'target_obj_pose-val :lambda-list '(m))
(cl:defmethod target_obj_pose-val ((m <SymbolGroundingGraspBasePose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:target_obj_pose-val is deprecated.  Use srs_symbolic_grounding-srv:target_obj_pose instead.")
  (target_obj_pose m))

(cl:ensure-generic-function 'parent_obj_geometry-val :lambda-list '(m))
(cl:defmethod parent_obj_geometry-val ((m <SymbolGroundingGraspBasePose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:parent_obj_geometry-val is deprecated.  Use srs_symbolic_grounding-srv:parent_obj_geometry instead.")
  (parent_obj_geometry m))

(cl:ensure-generic-function 'furniture_geometry_list-val :lambda-list '(m))
(cl:defmethod furniture_geometry_list-val ((m <SymbolGroundingGraspBasePose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:furniture_geometry_list-val is deprecated.  Use srs_symbolic_grounding-srv:furniture_geometry_list instead.")
  (furniture_geometry_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SymbolGroundingGraspBasePose-request>) ostream)
  "Serializes a message object of type '<SymbolGroundingGraspBasePose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_obj_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'parent_obj_geometry) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'furniture_geometry_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'furniture_geometry_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SymbolGroundingGraspBasePose-request>) istream)
  "Deserializes a message object of type '<SymbolGroundingGraspBasePose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_obj_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'parent_obj_geometry) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'furniture_geometry_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'furniture_geometry_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'srs_symbolic_grounding-msg:SRSSpatialInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SymbolGroundingGraspBasePose-request>)))
  "Returns string type for a service object of type '<SymbolGroundingGraspBasePose-request>"
  "srs_symbolic_grounding/SymbolGroundingGraspBasePoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SymbolGroundingGraspBasePose-request)))
  "Returns string type for a service object of type 'SymbolGroundingGraspBasePose-request"
  "srs_symbolic_grounding/SymbolGroundingGraspBasePoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SymbolGroundingGraspBasePose-request>)))
  "Returns md5sum for a message object of type '<SymbolGroundingGraspBasePose-request>"
  "80365e2dd1e4f246fb4542bfaa7f0518")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SymbolGroundingGraspBasePose-request)))
  "Returns md5sum for a message object of type 'SymbolGroundingGraspBasePose-request"
  "80365e2dd1e4f246fb4542bfaa7f0518")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SymbolGroundingGraspBasePose-request>)))
  "Returns full string definition for message of type '<SymbolGroundingGraspBasePose-request>"
  (cl:format cl:nil "geometry_msgs/Pose target_obj_pose~%SRSSpatialInfo parent_obj_geometry~%SRSSpatialInfo[] furniture_geometry_list~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: srs_symbolic_grounding/SRSSpatialInfo~%# Point point~%# Orientation angles~%float32 l~%float32 w~%float32 h~%~%geometry_msgs/Pose pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SymbolGroundingGraspBasePose-request)))
  "Returns full string definition for message of type 'SymbolGroundingGraspBasePose-request"
  (cl:format cl:nil "geometry_msgs/Pose target_obj_pose~%SRSSpatialInfo parent_obj_geometry~%SRSSpatialInfo[] furniture_geometry_list~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: srs_symbolic_grounding/SRSSpatialInfo~%# Point point~%# Orientation angles~%float32 l~%float32 w~%float32 h~%~%geometry_msgs/Pose pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SymbolGroundingGraspBasePose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_obj_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'parent_obj_geometry))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'furniture_geometry_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SymbolGroundingGraspBasePose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SymbolGroundingGraspBasePose-request
    (cl:cons ':target_obj_pose (target_obj_pose msg))
    (cl:cons ':parent_obj_geometry (parent_obj_geometry msg))
    (cl:cons ':furniture_geometry_list (furniture_geometry_list msg))
))
;//! \htmlinclude SymbolGroundingGraspBasePose-response.msg.html

(cl:defclass <SymbolGroundingGraspBasePose-response> (roslisp-msg-protocol:ros-message)
  ((obstacle_check
    :reader obstacle_check
    :initarg :obstacle_check
    :type cl:boolean
    :initform cl:nil)
   (reach
    :reader reach
    :initarg :reach
    :type cl:float
    :initform 0.0)
   (grasp_base_pose
    :reader grasp_base_pose
    :initarg :grasp_base_pose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D)))
)

(cl:defclass SymbolGroundingGraspBasePose-response (<SymbolGroundingGraspBasePose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SymbolGroundingGraspBasePose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SymbolGroundingGraspBasePose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_symbolic_grounding-srv:<SymbolGroundingGraspBasePose-response> is deprecated: use srs_symbolic_grounding-srv:SymbolGroundingGraspBasePose-response instead.")))

(cl:ensure-generic-function 'obstacle_check-val :lambda-list '(m))
(cl:defmethod obstacle_check-val ((m <SymbolGroundingGraspBasePose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:obstacle_check-val is deprecated.  Use srs_symbolic_grounding-srv:obstacle_check instead.")
  (obstacle_check m))

(cl:ensure-generic-function 'reach-val :lambda-list '(m))
(cl:defmethod reach-val ((m <SymbolGroundingGraspBasePose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:reach-val is deprecated.  Use srs_symbolic_grounding-srv:reach instead.")
  (reach m))

(cl:ensure-generic-function 'grasp_base_pose-val :lambda-list '(m))
(cl:defmethod grasp_base_pose-val ((m <SymbolGroundingGraspBasePose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:grasp_base_pose-val is deprecated.  Use srs_symbolic_grounding-srv:grasp_base_pose instead.")
  (grasp_base_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SymbolGroundingGraspBasePose-response>) ostream)
  "Serializes a message object of type '<SymbolGroundingGraspBasePose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'obstacle_check) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'reach))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'grasp_base_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SymbolGroundingGraspBasePose-response>) istream)
  "Deserializes a message object of type '<SymbolGroundingGraspBasePose-response>"
    (cl:setf (cl:slot-value msg 'obstacle_check) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reach) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'grasp_base_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SymbolGroundingGraspBasePose-response>)))
  "Returns string type for a service object of type '<SymbolGroundingGraspBasePose-response>"
  "srs_symbolic_grounding/SymbolGroundingGraspBasePoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SymbolGroundingGraspBasePose-response)))
  "Returns string type for a service object of type 'SymbolGroundingGraspBasePose-response"
  "srs_symbolic_grounding/SymbolGroundingGraspBasePoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SymbolGroundingGraspBasePose-response>)))
  "Returns md5sum for a message object of type '<SymbolGroundingGraspBasePose-response>"
  "80365e2dd1e4f246fb4542bfaa7f0518")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SymbolGroundingGraspBasePose-response)))
  "Returns md5sum for a message object of type 'SymbolGroundingGraspBasePose-response"
  "80365e2dd1e4f246fb4542bfaa7f0518")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SymbolGroundingGraspBasePose-response>)))
  "Returns full string definition for message of type '<SymbolGroundingGraspBasePose-response>"
  (cl:format cl:nil "bool obstacle_check~%float32 reach~%geometry_msgs/Pose2D grasp_base_pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SymbolGroundingGraspBasePose-response)))
  "Returns full string definition for message of type 'SymbolGroundingGraspBasePose-response"
  (cl:format cl:nil "bool obstacle_check~%float32 reach~%geometry_msgs/Pose2D grasp_base_pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SymbolGroundingGraspBasePose-response>))
  (cl:+ 0
     1
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'grasp_base_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SymbolGroundingGraspBasePose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SymbolGroundingGraspBasePose-response
    (cl:cons ':obstacle_check (obstacle_check msg))
    (cl:cons ':reach (reach msg))
    (cl:cons ':grasp_base_pose (grasp_base_pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SymbolGroundingGraspBasePose)))
  'SymbolGroundingGraspBasePose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SymbolGroundingGraspBasePose)))
  'SymbolGroundingGraspBasePose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SymbolGroundingGraspBasePose)))
  "Returns string type for a service object of type '<SymbolGroundingGraspBasePose>"
  "srs_symbolic_grounding/SymbolGroundingGraspBasePose")