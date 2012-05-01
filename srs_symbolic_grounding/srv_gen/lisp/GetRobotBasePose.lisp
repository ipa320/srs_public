; Auto-generated. Do not edit!


(cl:in-package srs_symbolic_grounding-srv)


;//! \htmlinclude GetRobotBasePose-request.msg.html

(cl:defclass <GetRobotBasePose-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetRobotBasePose-request (<GetRobotBasePose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotBasePose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotBasePose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_symbolic_grounding-srv:<GetRobotBasePose-request> is deprecated: use srs_symbolic_grounding-srv:GetRobotBasePose-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotBasePose-request>) ostream)
  "Serializes a message object of type '<GetRobotBasePose-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotBasePose-request>) istream)
  "Deserializes a message object of type '<GetRobotBasePose-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotBasePose-request>)))
  "Returns string type for a service object of type '<GetRobotBasePose-request>"
  "srs_symbolic_grounding/GetRobotBasePoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotBasePose-request)))
  "Returns string type for a service object of type 'GetRobotBasePose-request"
  "srs_symbolic_grounding/GetRobotBasePoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotBasePose-request>)))
  "Returns md5sum for a message object of type '<GetRobotBasePose-request>"
  "ea3141e104f2c92b1676fbd6e8c1a602")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotBasePose-request)))
  "Returns md5sum for a message object of type 'GetRobotBasePose-request"
  "ea3141e104f2c92b1676fbd6e8c1a602")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotBasePose-request>)))
  "Returns full string definition for message of type '<GetRobotBasePose-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotBasePose-request)))
  "Returns full string definition for message of type 'GetRobotBasePose-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotBasePose-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotBasePose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotBasePose-request
))
;//! \htmlinclude GetRobotBasePose-response.msg.html

(cl:defclass <GetRobotBasePose-response> (roslisp-msg-protocol:ros-message)
  ((rb_pose
    :reader rb_pose
    :initarg :rb_pose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D)))
)

(cl:defclass GetRobotBasePose-response (<GetRobotBasePose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotBasePose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotBasePose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_symbolic_grounding-srv:<GetRobotBasePose-response> is deprecated: use srs_symbolic_grounding-srv:GetRobotBasePose-response instead.")))

(cl:ensure-generic-function 'rb_pose-val :lambda-list '(m))
(cl:defmethod rb_pose-val ((m <GetRobotBasePose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:rb_pose-val is deprecated.  Use srs_symbolic_grounding-srv:rb_pose instead.")
  (rb_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotBasePose-response>) ostream)
  "Serializes a message object of type '<GetRobotBasePose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rb_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotBasePose-response>) istream)
  "Deserializes a message object of type '<GetRobotBasePose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rb_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotBasePose-response>)))
  "Returns string type for a service object of type '<GetRobotBasePose-response>"
  "srs_symbolic_grounding/GetRobotBasePoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotBasePose-response)))
  "Returns string type for a service object of type 'GetRobotBasePose-response"
  "srs_symbolic_grounding/GetRobotBasePoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotBasePose-response>)))
  "Returns md5sum for a message object of type '<GetRobotBasePose-response>"
  "ea3141e104f2c92b1676fbd6e8c1a602")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotBasePose-response)))
  "Returns md5sum for a message object of type 'GetRobotBasePose-response"
  "ea3141e104f2c92b1676fbd6e8c1a602")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotBasePose-response>)))
  "Returns full string definition for message of type '<GetRobotBasePose-response>"
  (cl:format cl:nil "geometry_msgs/Pose2D rb_pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotBasePose-response)))
  "Returns full string definition for message of type 'GetRobotBasePose-response"
  (cl:format cl:nil "geometry_msgs/Pose2D rb_pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotBasePose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rb_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotBasePose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotBasePose-response
    (cl:cons ':rb_pose (rb_pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetRobotBasePose)))
  'GetRobotBasePose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetRobotBasePose)))
  'GetRobotBasePose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotBasePose)))
  "Returns string type for a service object of type '<GetRobotBasePose>"
  "srs_symbolic_grounding/GetRobotBasePose")