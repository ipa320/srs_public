; Auto-generated. Do not edit!


(cl:in-package srs_grasping-srv)


;//! \htmlinclude GetGraspsFromPosition-request.msg.html

(cl:defclass <GetGraspsFromPosition-request> (roslisp-msg-protocol:ros-message)
  ((object_id
    :reader object_id
    :initarg :object_id
    :type cl:integer
    :initform 0)
   (object_pose
    :reader object_pose
    :initarg :object_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass GetGraspsFromPosition-request (<GetGraspsFromPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetGraspsFromPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetGraspsFromPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_grasping-srv:<GetGraspsFromPosition-request> is deprecated: use srs_grasping-srv:GetGraspsFromPosition-request instead.")))

(cl:ensure-generic-function 'object_id-val :lambda-list '(m))
(cl:defmethod object_id-val ((m <GetGraspsFromPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_grasping-srv:object_id-val is deprecated.  Use srs_grasping-srv:object_id instead.")
  (object_id m))

(cl:ensure-generic-function 'object_pose-val :lambda-list '(m))
(cl:defmethod object_pose-val ((m <GetGraspsFromPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_grasping-srv:object_pose-val is deprecated.  Use srs_grasping-srv:object_pose instead.")
  (object_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetGraspsFromPosition-request>) ostream)
  "Serializes a message object of type '<GetGraspsFromPosition-request>"
  (cl:let* ((signed (cl:slot-value msg 'object_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetGraspsFromPosition-request>) istream)
  "Deserializes a message object of type '<GetGraspsFromPosition-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetGraspsFromPosition-request>)))
  "Returns string type for a service object of type '<GetGraspsFromPosition-request>"
  "srs_grasping/GetGraspsFromPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGraspsFromPosition-request)))
  "Returns string type for a service object of type 'GetGraspsFromPosition-request"
  "srs_grasping/GetGraspsFromPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetGraspsFromPosition-request>)))
  "Returns md5sum for a message object of type '<GetGraspsFromPosition-request>"
  "df8bf0b028f9699d0ff4b300630fff51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetGraspsFromPosition-request)))
  "Returns md5sum for a message object of type 'GetGraspsFromPosition-request"
  "df8bf0b028f9699d0ff4b300630fff51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetGraspsFromPosition-request>)))
  "Returns full string definition for message of type '<GetGraspsFromPosition-request>"
  (cl:format cl:nil "int32 object_id~%geometry_msgs/Pose object_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetGraspsFromPosition-request)))
  "Returns full string definition for message of type 'GetGraspsFromPosition-request"
  (cl:format cl:nil "int32 object_id~%geometry_msgs/Pose object_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetGraspsFromPosition-request>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetGraspsFromPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetGraspsFromPosition-request
    (cl:cons ':object_id (object_id msg))
    (cl:cons ':object_pose (object_pose msg))
))
;//! \htmlinclude GetGraspsFromPosition-response.msg.html

(cl:defclass <GetGraspsFromPosition-response> (roslisp-msg-protocol:ros-message)
  ((feasible_grasp_available
    :reader feasible_grasp_available
    :initarg :feasible_grasp_available
    :type cl:boolean
    :initform cl:nil)
   (grasp_configuration
    :reader grasp_configuration
    :initarg :grasp_configuration
    :type (cl:vector srs_msgs-msg:GraspSubConfiguration)
   :initform (cl:make-array 0 :element-type 'srs_msgs-msg:GraspSubConfiguration :initial-element (cl:make-instance 'srs_msgs-msg:GraspSubConfiguration))))
)

(cl:defclass GetGraspsFromPosition-response (<GetGraspsFromPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetGraspsFromPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetGraspsFromPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_grasping-srv:<GetGraspsFromPosition-response> is deprecated: use srs_grasping-srv:GetGraspsFromPosition-response instead.")))

(cl:ensure-generic-function 'feasible_grasp_available-val :lambda-list '(m))
(cl:defmethod feasible_grasp_available-val ((m <GetGraspsFromPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_grasping-srv:feasible_grasp_available-val is deprecated.  Use srs_grasping-srv:feasible_grasp_available instead.")
  (feasible_grasp_available m))

(cl:ensure-generic-function 'grasp_configuration-val :lambda-list '(m))
(cl:defmethod grasp_configuration-val ((m <GetGraspsFromPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_grasping-srv:grasp_configuration-val is deprecated.  Use srs_grasping-srv:grasp_configuration instead.")
  (grasp_configuration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetGraspsFromPosition-response>) ostream)
  "Serializes a message object of type '<GetGraspsFromPosition-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'feasible_grasp_available) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'grasp_configuration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'grasp_configuration))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetGraspsFromPosition-response>) istream)
  "Deserializes a message object of type '<GetGraspsFromPosition-response>"
    (cl:setf (cl:slot-value msg 'feasible_grasp_available) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'grasp_configuration) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'grasp_configuration)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'srs_msgs-msg:GraspSubConfiguration))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetGraspsFromPosition-response>)))
  "Returns string type for a service object of type '<GetGraspsFromPosition-response>"
  "srs_grasping/GetGraspsFromPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGraspsFromPosition-response)))
  "Returns string type for a service object of type 'GetGraspsFromPosition-response"
  "srs_grasping/GetGraspsFromPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetGraspsFromPosition-response>)))
  "Returns md5sum for a message object of type '<GetGraspsFromPosition-response>"
  "df8bf0b028f9699d0ff4b300630fff51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetGraspsFromPosition-response)))
  "Returns md5sum for a message object of type 'GetGraspsFromPosition-response"
  "df8bf0b028f9699d0ff4b300630fff51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetGraspsFromPosition-response>)))
  "Returns full string definition for message of type '<GetGraspsFromPosition-response>"
  (cl:format cl:nil "bool feasible_grasp_available~%srs_msgs/GraspSubConfiguration[] grasp_configuration~%~%~%================================================================================~%MSG: srs_msgs/GraspSubConfiguration~%float64[] sdh_joint_values~%geometry_msgs/Pose grasp~%geometry_msgs/Pose pre_grasp~%string category~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetGraspsFromPosition-response)))
  "Returns full string definition for message of type 'GetGraspsFromPosition-response"
  (cl:format cl:nil "bool feasible_grasp_available~%srs_msgs/GraspSubConfiguration[] grasp_configuration~%~%~%================================================================================~%MSG: srs_msgs/GraspSubConfiguration~%float64[] sdh_joint_values~%geometry_msgs/Pose grasp~%geometry_msgs/Pose pre_grasp~%string category~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetGraspsFromPosition-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'grasp_configuration) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetGraspsFromPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetGraspsFromPosition-response
    (cl:cons ':feasible_grasp_available (feasible_grasp_available msg))
    (cl:cons ':grasp_configuration (grasp_configuration msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetGraspsFromPosition)))
  'GetGraspsFromPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetGraspsFromPosition)))
  'GetGraspsFromPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGraspsFromPosition)))
  "Returns string type for a service object of type '<GetGraspsFromPosition>"
  "srs_grasping/GetGraspsFromPosition")