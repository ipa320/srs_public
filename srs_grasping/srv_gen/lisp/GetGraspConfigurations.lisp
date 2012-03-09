; Auto-generated. Do not edit!


(cl:in-package srs_grasping-srv)


;//! \htmlinclude GetGraspConfigurations-request.msg.html

(cl:defclass <GetGraspConfigurations-request> (roslisp-msg-protocol:ros-message)
  ((object_id
    :reader object_id
    :initarg :object_id
    :type cl:integer
    :initform 0))
)

(cl:defclass GetGraspConfigurations-request (<GetGraspConfigurations-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetGraspConfigurations-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetGraspConfigurations-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_grasping-srv:<GetGraspConfigurations-request> is deprecated: use srs_grasping-srv:GetGraspConfigurations-request instead.")))

(cl:ensure-generic-function 'object_id-val :lambda-list '(m))
(cl:defmethod object_id-val ((m <GetGraspConfigurations-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_grasping-srv:object_id-val is deprecated.  Use srs_grasping-srv:object_id instead.")
  (object_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetGraspConfigurations-request>) ostream)
  "Serializes a message object of type '<GetGraspConfigurations-request>"
  (cl:let* ((signed (cl:slot-value msg 'object_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetGraspConfigurations-request>) istream)
  "Deserializes a message object of type '<GetGraspConfigurations-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetGraspConfigurations-request>)))
  "Returns string type for a service object of type '<GetGraspConfigurations-request>"
  "srs_grasping/GetGraspConfigurationsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGraspConfigurations-request)))
  "Returns string type for a service object of type 'GetGraspConfigurations-request"
  "srs_grasping/GetGraspConfigurationsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetGraspConfigurations-request>)))
  "Returns md5sum for a message object of type '<GetGraspConfigurations-request>"
  "37caef0185f2ced87cd79f4e67bb74e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetGraspConfigurations-request)))
  "Returns md5sum for a message object of type 'GetGraspConfigurations-request"
  "37caef0185f2ced87cd79f4e67bb74e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetGraspConfigurations-request>)))
  "Returns full string definition for message of type '<GetGraspConfigurations-request>"
  (cl:format cl:nil "int32 object_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetGraspConfigurations-request)))
  "Returns full string definition for message of type 'GetGraspConfigurations-request"
  (cl:format cl:nil "int32 object_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetGraspConfigurations-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetGraspConfigurations-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetGraspConfigurations-request
    (cl:cons ':object_id (object_id msg))
))
;//! \htmlinclude GetGraspConfigurations-response.msg.html

(cl:defclass <GetGraspConfigurations-response> (roslisp-msg-protocol:ros-message)
  ((grasp_configuration
    :reader grasp_configuration
    :initarg :grasp_configuration
    :type (cl:vector srs_msgs-msg:GraspConfiguration)
   :initform (cl:make-array 0 :element-type 'srs_msgs-msg:GraspConfiguration :initial-element (cl:make-instance 'srs_msgs-msg:GraspConfiguration))))
)

(cl:defclass GetGraspConfigurations-response (<GetGraspConfigurations-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetGraspConfigurations-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetGraspConfigurations-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_grasping-srv:<GetGraspConfigurations-response> is deprecated: use srs_grasping-srv:GetGraspConfigurations-response instead.")))

(cl:ensure-generic-function 'grasp_configuration-val :lambda-list '(m))
(cl:defmethod grasp_configuration-val ((m <GetGraspConfigurations-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_grasping-srv:grasp_configuration-val is deprecated.  Use srs_grasping-srv:grasp_configuration instead.")
  (grasp_configuration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetGraspConfigurations-response>) ostream)
  "Serializes a message object of type '<GetGraspConfigurations-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'grasp_configuration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'grasp_configuration))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetGraspConfigurations-response>) istream)
  "Deserializes a message object of type '<GetGraspConfigurations-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'grasp_configuration) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'grasp_configuration)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'srs_msgs-msg:GraspConfiguration))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetGraspConfigurations-response>)))
  "Returns string type for a service object of type '<GetGraspConfigurations-response>"
  "srs_grasping/GetGraspConfigurationsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGraspConfigurations-response)))
  "Returns string type for a service object of type 'GetGraspConfigurations-response"
  "srs_grasping/GetGraspConfigurationsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetGraspConfigurations-response>)))
  "Returns md5sum for a message object of type '<GetGraspConfigurations-response>"
  "37caef0185f2ced87cd79f4e67bb74e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetGraspConfigurations-response)))
  "Returns md5sum for a message object of type 'GetGraspConfigurations-response"
  "37caef0185f2ced87cd79f4e67bb74e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetGraspConfigurations-response>)))
  "Returns full string definition for message of type '<GetGraspConfigurations-response>"
  (cl:format cl:nil "srs_msgs/GraspConfiguration[] grasp_configuration~%~%~%================================================================================~%MSG: srs_msgs/GraspConfiguration~%int32 object_id~%string hand_type~%float64[] sdh_joint_values~%string target_link #link which should be moved to pre_grasp (e.g. sdh_palm_link)~%geometry_msgs/PoseStamped pre_grasp~%geometry_msgs/PoseStamped grasp~%string category~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetGraspConfigurations-response)))
  "Returns full string definition for message of type 'GetGraspConfigurations-response"
  (cl:format cl:nil "srs_msgs/GraspConfiguration[] grasp_configuration~%~%~%================================================================================~%MSG: srs_msgs/GraspConfiguration~%int32 object_id~%string hand_type~%float64[] sdh_joint_values~%string target_link #link which should be moved to pre_grasp (e.g. sdh_palm_link)~%geometry_msgs/PoseStamped pre_grasp~%geometry_msgs/PoseStamped grasp~%string category~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetGraspConfigurations-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'grasp_configuration) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetGraspConfigurations-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetGraspConfigurations-response
    (cl:cons ':grasp_configuration (grasp_configuration msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetGraspConfigurations)))
  'GetGraspConfigurations-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetGraspConfigurations)))
  'GetGraspConfigurations-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetGraspConfigurations)))
  "Returns string type for a service object of type '<GetGraspConfigurations>"
  "srs_grasping/GetGraspConfigurations")