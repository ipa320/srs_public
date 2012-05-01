; Auto-generated. Do not edit!


(cl:in-package srs_symbolic_grounding-srv)


;//! \htmlinclude SymbolGroundingPrimitiveBasePose-request.msg.html

(cl:defclass <SymbolGroundingPrimitiveBasePose-request> (roslisp-msg-protocol:ros-message)
  ((obj_pose
    :reader obj_pose
    :initarg :obj_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass SymbolGroundingPrimitiveBasePose-request (<SymbolGroundingPrimitiveBasePose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SymbolGroundingPrimitiveBasePose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SymbolGroundingPrimitiveBasePose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_symbolic_grounding-srv:<SymbolGroundingPrimitiveBasePose-request> is deprecated: use srs_symbolic_grounding-srv:SymbolGroundingPrimitiveBasePose-request instead.")))

(cl:ensure-generic-function 'obj_pose-val :lambda-list '(m))
(cl:defmethod obj_pose-val ((m <SymbolGroundingPrimitiveBasePose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:obj_pose-val is deprecated.  Use srs_symbolic_grounding-srv:obj_pose instead.")
  (obj_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SymbolGroundingPrimitiveBasePose-request>) ostream)
  "Serializes a message object of type '<SymbolGroundingPrimitiveBasePose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obj_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SymbolGroundingPrimitiveBasePose-request>) istream)
  "Deserializes a message object of type '<SymbolGroundingPrimitiveBasePose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obj_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SymbolGroundingPrimitiveBasePose-request>)))
  "Returns string type for a service object of type '<SymbolGroundingPrimitiveBasePose-request>"
  "srs_symbolic_grounding/SymbolGroundingPrimitiveBasePoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SymbolGroundingPrimitiveBasePose-request)))
  "Returns string type for a service object of type 'SymbolGroundingPrimitiveBasePose-request"
  "srs_symbolic_grounding/SymbolGroundingPrimitiveBasePoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SymbolGroundingPrimitiveBasePose-request>)))
  "Returns md5sum for a message object of type '<SymbolGroundingPrimitiveBasePose-request>"
  "9e95d0d21a8416a1a1a25043b4b05197")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SymbolGroundingPrimitiveBasePose-request)))
  "Returns md5sum for a message object of type 'SymbolGroundingPrimitiveBasePose-request"
  "9e95d0d21a8416a1a1a25043b4b05197")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SymbolGroundingPrimitiveBasePose-request>)))
  "Returns full string definition for message of type '<SymbolGroundingPrimitiveBasePose-request>"
  (cl:format cl:nil "geometry_msgs/Pose obj_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SymbolGroundingPrimitiveBasePose-request)))
  "Returns full string definition for message of type 'SymbolGroundingPrimitiveBasePose-request"
  (cl:format cl:nil "geometry_msgs/Pose obj_pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SymbolGroundingPrimitiveBasePose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obj_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SymbolGroundingPrimitiveBasePose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SymbolGroundingPrimitiveBasePose-request
    (cl:cons ':obj_pose (obj_pose msg))
))
;//! \htmlinclude SymbolGroundingPrimitiveBasePose-response.msg.html

(cl:defclass <SymbolGroundingPrimitiveBasePose-response> (roslisp-msg-protocol:ros-message)
  ((pbp
    :reader pbp
    :initarg :pbp
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D)))
)

(cl:defclass SymbolGroundingPrimitiveBasePose-response (<SymbolGroundingPrimitiveBasePose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SymbolGroundingPrimitiveBasePose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SymbolGroundingPrimitiveBasePose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_symbolic_grounding-srv:<SymbolGroundingPrimitiveBasePose-response> is deprecated: use srs_symbolic_grounding-srv:SymbolGroundingPrimitiveBasePose-response instead.")))

(cl:ensure-generic-function 'pbp-val :lambda-list '(m))
(cl:defmethod pbp-val ((m <SymbolGroundingPrimitiveBasePose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:pbp-val is deprecated.  Use srs_symbolic_grounding-srv:pbp instead.")
  (pbp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SymbolGroundingPrimitiveBasePose-response>) ostream)
  "Serializes a message object of type '<SymbolGroundingPrimitiveBasePose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pbp) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SymbolGroundingPrimitiveBasePose-response>) istream)
  "Deserializes a message object of type '<SymbolGroundingPrimitiveBasePose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pbp) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SymbolGroundingPrimitiveBasePose-response>)))
  "Returns string type for a service object of type '<SymbolGroundingPrimitiveBasePose-response>"
  "srs_symbolic_grounding/SymbolGroundingPrimitiveBasePoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SymbolGroundingPrimitiveBasePose-response)))
  "Returns string type for a service object of type 'SymbolGroundingPrimitiveBasePose-response"
  "srs_symbolic_grounding/SymbolGroundingPrimitiveBasePoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SymbolGroundingPrimitiveBasePose-response>)))
  "Returns md5sum for a message object of type '<SymbolGroundingPrimitiveBasePose-response>"
  "9e95d0d21a8416a1a1a25043b4b05197")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SymbolGroundingPrimitiveBasePose-response)))
  "Returns md5sum for a message object of type 'SymbolGroundingPrimitiveBasePose-response"
  "9e95d0d21a8416a1a1a25043b4b05197")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SymbolGroundingPrimitiveBasePose-response>)))
  "Returns full string definition for message of type '<SymbolGroundingPrimitiveBasePose-response>"
  (cl:format cl:nil "geometry_msgs/Pose2D pbp~%~%~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SymbolGroundingPrimitiveBasePose-response)))
  "Returns full string definition for message of type 'SymbolGroundingPrimitiveBasePose-response"
  (cl:format cl:nil "geometry_msgs/Pose2D pbp~%~%~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SymbolGroundingPrimitiveBasePose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pbp))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SymbolGroundingPrimitiveBasePose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SymbolGroundingPrimitiveBasePose-response
    (cl:cons ':pbp (pbp msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SymbolGroundingPrimitiveBasePose)))
  'SymbolGroundingPrimitiveBasePose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SymbolGroundingPrimitiveBasePose)))
  'SymbolGroundingPrimitiveBasePose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SymbolGroundingPrimitiveBasePose)))
  "Returns string type for a service object of type '<SymbolGroundingPrimitiveBasePose>"
  "srs_symbolic_grounding/SymbolGroundingPrimitiveBasePose")