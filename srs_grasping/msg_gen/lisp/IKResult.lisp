; Auto-generated. Do not edit!


(cl:in-package srs_grasping-msg)


;//! \htmlinclude IKResult.msg.html

(cl:defclass <IKResult> (roslisp-msg-protocol:ros-message)
  ((pre_position
    :reader pre_position
    :initarg :pre_position
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (g_position
    :reader g_position
    :initarg :g_position
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass IKResult (<IKResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IKResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IKResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_grasping-msg:<IKResult> is deprecated: use srs_grasping-msg:IKResult instead.")))

(cl:ensure-generic-function 'pre_position-val :lambda-list '(m))
(cl:defmethod pre_position-val ((m <IKResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_grasping-msg:pre_position-val is deprecated.  Use srs_grasping-msg:pre_position instead.")
  (pre_position m))

(cl:ensure-generic-function 'g_position-val :lambda-list '(m))
(cl:defmethod g_position-val ((m <IKResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_grasping-msg:g_position-val is deprecated.  Use srs_grasping-msg:g_position instead.")
  (g_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IKResult>) ostream)
  "Serializes a message object of type '<IKResult>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pre_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'pre_position))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'g_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'g_position))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IKResult>) istream)
  "Deserializes a message object of type '<IKResult>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pre_position) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pre_position)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'g_position) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'g_position)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IKResult>)))
  "Returns string type for a message object of type '<IKResult>"
  "srs_grasping/IKResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IKResult)))
  "Returns string type for a message object of type 'IKResult"
  "srs_grasping/IKResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IKResult>)))
  "Returns md5sum for a message object of type '<IKResult>"
  "c210c8523fd6ad4179d213c0a49f1504")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IKResult)))
  "Returns md5sum for a message object of type 'IKResult"
  "c210c8523fd6ad4179d213c0a49f1504")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IKResult>)))
  "Returns full string definition for message of type '<IKResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%float64[] pre_position~%float64[] g_position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IKResult)))
  "Returns full string definition for message of type 'IKResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%float64[] pre_position~%float64[] g_position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IKResult>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pre_position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'g_position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IKResult>))
  "Converts a ROS message object to a list"
  (cl:list 'IKResult
    (cl:cons ':pre_position (pre_position msg))
    (cl:cons ':g_position (g_position msg))
))
