; Auto-generated. Do not edit!


(cl:in-package srs_symbolic_grounding-msg)


;//! \htmlinclude FurnitureGeometry.msg.html

(cl:defclass <FurnitureGeometry> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (l
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
    :initform 0.0))
)

(cl:defclass FurnitureGeometry (<FurnitureGeometry>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FurnitureGeometry>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FurnitureGeometry)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_symbolic_grounding-msg:<FurnitureGeometry> is deprecated: use srs_symbolic_grounding-msg:FurnitureGeometry instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <FurnitureGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-msg:pose-val is deprecated.  Use srs_symbolic_grounding-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'l-val :lambda-list '(m))
(cl:defmethod l-val ((m <FurnitureGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-msg:l-val is deprecated.  Use srs_symbolic_grounding-msg:l instead.")
  (l m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <FurnitureGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-msg:w-val is deprecated.  Use srs_symbolic_grounding-msg:w instead.")
  (w m))

(cl:ensure-generic-function 'h-val :lambda-list '(m))
(cl:defmethod h-val ((m <FurnitureGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-msg:h-val is deprecated.  Use srs_symbolic_grounding-msg:h instead.")
  (h m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FurnitureGeometry>) ostream)
  "Serializes a message object of type '<FurnitureGeometry>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FurnitureGeometry>) istream)
  "Deserializes a message object of type '<FurnitureGeometry>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FurnitureGeometry>)))
  "Returns string type for a message object of type '<FurnitureGeometry>"
  "srs_symbolic_grounding/FurnitureGeometry")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FurnitureGeometry)))
  "Returns string type for a message object of type 'FurnitureGeometry"
  "srs_symbolic_grounding/FurnitureGeometry")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FurnitureGeometry>)))
  "Returns md5sum for a message object of type '<FurnitureGeometry>"
  "9eadcd72801a473181cb7b090c264a15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FurnitureGeometry)))
  "Returns md5sum for a message object of type 'FurnitureGeometry"
  "9eadcd72801a473181cb7b090c264a15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FurnitureGeometry>)))
  "Returns full string definition for message of type '<FurnitureGeometry>"
  (cl:format cl:nil "geometry_msgs/Pose2D pose~%float32 l~%float32 w~%float32 h~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FurnitureGeometry)))
  "Returns full string definition for message of type 'FurnitureGeometry"
  (cl:format cl:nil "geometry_msgs/Pose2D pose~%float32 l~%float32 w~%float32 h~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FurnitureGeometry>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FurnitureGeometry>))
  "Converts a ROS message object to a list"
  (cl:list 'FurnitureGeometry
    (cl:cons ':pose (pose msg))
    (cl:cons ':l (l msg))
    (cl:cons ':w (w msg))
    (cl:cons ':h (h msg))
))
