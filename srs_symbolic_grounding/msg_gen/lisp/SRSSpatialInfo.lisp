; Auto-generated. Do not edit!


(cl:in-package srs_symbolic_grounding-msg)


;//! \htmlinclude SRSSpatialInfo.msg.html

(cl:defclass <SRSSpatialInfo> (roslisp-msg-protocol:ros-message)
  ((l
    :reader l
    :initarg :l
    :type cl:float
    :initform 0.0)
   (w
    :reader w
    :initarg :w
    :type cl:float
    :initform 0.0)
   (h
    :reader h
    :initarg :h
    :type cl:float
    :initform 0.0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass SRSSpatialInfo (<SRSSpatialInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SRSSpatialInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SRSSpatialInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_symbolic_grounding-msg:<SRSSpatialInfo> is deprecated: use srs_symbolic_grounding-msg:SRSSpatialInfo instead.")))

(cl:ensure-generic-function 'l-val :lambda-list '(m))
(cl:defmethod l-val ((m <SRSSpatialInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-msg:l-val is deprecated.  Use srs_symbolic_grounding-msg:l instead.")
  (l m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <SRSSpatialInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-msg:w-val is deprecated.  Use srs_symbolic_grounding-msg:w instead.")
  (w m))

(cl:ensure-generic-function 'h-val :lambda-list '(m))
(cl:defmethod h-val ((m <SRSSpatialInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-msg:h-val is deprecated.  Use srs_symbolic_grounding-msg:h instead.")
  (h m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <SRSSpatialInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-msg:pose-val is deprecated.  Use srs_symbolic_grounding-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SRSSpatialInfo>) ostream)
  "Serializes a message object of type '<SRSSpatialInfo>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'l))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'h))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SRSSpatialInfo>) istream)
  "Deserializes a message object of type '<SRSSpatialInfo>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'l) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'w) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'h) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SRSSpatialInfo>)))
  "Returns string type for a message object of type '<SRSSpatialInfo>"
  "srs_symbolic_grounding/SRSSpatialInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SRSSpatialInfo)))
  "Returns string type for a message object of type 'SRSSpatialInfo"
  "srs_symbolic_grounding/SRSSpatialInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SRSSpatialInfo>)))
  "Returns md5sum for a message object of type '<SRSSpatialInfo>"
  "258ee9bd984fca3e863d2c8404dc39e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SRSSpatialInfo)))
  "Returns md5sum for a message object of type 'SRSSpatialInfo"
  "258ee9bd984fca3e863d2c8404dc39e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SRSSpatialInfo>)))
  "Returns full string definition for message of type '<SRSSpatialInfo>"
  (cl:format cl:nil "# Point point~%# Orientation angles~%float32 l~%float32 w~%float32 h~%~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SRSSpatialInfo)))
  "Returns full string definition for message of type 'SRSSpatialInfo"
  (cl:format cl:nil "# Point point~%# Orientation angles~%float32 l~%float32 w~%float32 h~%~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SRSSpatialInfo>))
  (cl:+ 0
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SRSSpatialInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'SRSSpatialInfo
    (cl:cons ':l (l msg))
    (cl:cons ':w (w msg))
    (cl:cons ':h (h msg))
    (cl:cons ':pose (pose msg))
))
