; Auto-generated. Do not edit!


(cl:in-package srs_symbolic_grounding-srv)


;//! \htmlinclude GetWorkspaceOnMap-request.msg.html

(cl:defclass <GetWorkspaceOnMap-request> (roslisp-msg-protocol:ros-message)
  ((map
    :reader map
    :initarg :map
    :type cl:string
    :initform "")
   (ifGeometryInfo
    :reader ifGeometryInfo
    :initarg :ifGeometryInfo
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetWorkspaceOnMap-request (<GetWorkspaceOnMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWorkspaceOnMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWorkspaceOnMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_symbolic_grounding-srv:<GetWorkspaceOnMap-request> is deprecated: use srs_symbolic_grounding-srv:GetWorkspaceOnMap-request instead.")))

(cl:ensure-generic-function 'map-val :lambda-list '(m))
(cl:defmethod map-val ((m <GetWorkspaceOnMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:map-val is deprecated.  Use srs_symbolic_grounding-srv:map instead.")
  (map m))

(cl:ensure-generic-function 'ifGeometryInfo-val :lambda-list '(m))
(cl:defmethod ifGeometryInfo-val ((m <GetWorkspaceOnMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:ifGeometryInfo-val is deprecated.  Use srs_symbolic_grounding-srv:ifGeometryInfo instead.")
  (ifGeometryInfo m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWorkspaceOnMap-request>) ostream)
  "Serializes a message object of type '<GetWorkspaceOnMap-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'map))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'map))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ifGeometryInfo) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWorkspaceOnMap-request>) istream)
  "Deserializes a message object of type '<GetWorkspaceOnMap-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'map) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'map) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'ifGeometryInfo) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWorkspaceOnMap-request>)))
  "Returns string type for a service object of type '<GetWorkspaceOnMap-request>"
  "srs_symbolic_grounding/GetWorkspaceOnMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWorkspaceOnMap-request)))
  "Returns string type for a service object of type 'GetWorkspaceOnMap-request"
  "srs_symbolic_grounding/GetWorkspaceOnMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWorkspaceOnMap-request>)))
  "Returns md5sum for a message object of type '<GetWorkspaceOnMap-request>"
  "4aac5a2e75ff4e1c2a3f288dcf79c97a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWorkspaceOnMap-request)))
  "Returns md5sum for a message object of type 'GetWorkspaceOnMap-request"
  "4aac5a2e75ff4e1c2a3f288dcf79c97a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWorkspaceOnMap-request>)))
  "Returns full string definition for message of type '<GetWorkspaceOnMap-request>"
  (cl:format cl:nil "~%~%~%~%~%~%string map~%~%~%~%~%bool ifGeometryInfo~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWorkspaceOnMap-request)))
  "Returns full string definition for message of type 'GetWorkspaceOnMap-request"
  (cl:format cl:nil "~%~%~%~%~%~%string map~%~%~%~%~%bool ifGeometryInfo~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWorkspaceOnMap-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'map))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWorkspaceOnMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWorkspaceOnMap-request
    (cl:cons ':map (map msg))
    (cl:cons ':ifGeometryInfo (ifGeometryInfo msg))
))
;//! \htmlinclude GetWorkspaceOnMap-response.msg.html

(cl:defclass <GetWorkspaceOnMap-response> (roslisp-msg-protocol:ros-message)
  ((objects
    :reader objects
    :initarg :objects
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (classesOfObjects
    :reader classesOfObjects
    :initarg :classesOfObjects
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (objectsInfo
    :reader objectsInfo
    :initarg :objectsInfo
    :type (cl:vector srs_symbolic_grounding-msg:SRSSpatialInfo)
   :initform (cl:make-array 0 :element-type 'srs_symbolic_grounding-msg:SRSSpatialInfo :initial-element (cl:make-instance 'srs_symbolic_grounding-msg:SRSSpatialInfo)))
   (houseHoldId
    :reader houseHoldId
    :initarg :houseHoldId
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass GetWorkspaceOnMap-response (<GetWorkspaceOnMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWorkspaceOnMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWorkspaceOnMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srs_symbolic_grounding-srv:<GetWorkspaceOnMap-response> is deprecated: use srs_symbolic_grounding-srv:GetWorkspaceOnMap-response instead.")))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <GetWorkspaceOnMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:objects-val is deprecated.  Use srs_symbolic_grounding-srv:objects instead.")
  (objects m))

(cl:ensure-generic-function 'classesOfObjects-val :lambda-list '(m))
(cl:defmethod classesOfObjects-val ((m <GetWorkspaceOnMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:classesOfObjects-val is deprecated.  Use srs_symbolic_grounding-srv:classesOfObjects instead.")
  (classesOfObjects m))

(cl:ensure-generic-function 'objectsInfo-val :lambda-list '(m))
(cl:defmethod objectsInfo-val ((m <GetWorkspaceOnMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:objectsInfo-val is deprecated.  Use srs_symbolic_grounding-srv:objectsInfo instead.")
  (objectsInfo m))

(cl:ensure-generic-function 'houseHoldId-val :lambda-list '(m))
(cl:defmethod houseHoldId-val ((m <GetWorkspaceOnMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srs_symbolic_grounding-srv:houseHoldId-val is deprecated.  Use srs_symbolic_grounding-srv:houseHoldId instead.")
  (houseHoldId m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWorkspaceOnMap-response>) ostream)
  "Serializes a message object of type '<GetWorkspaceOnMap-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'objects))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'classesOfObjects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'classesOfObjects))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objectsInfo))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'objectsInfo))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'houseHoldId))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'houseHoldId))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWorkspaceOnMap-response>) istream)
  "Deserializes a message object of type '<GetWorkspaceOnMap-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'classesOfObjects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'classesOfObjects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objectsInfo) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objectsInfo)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'srs_symbolic_grounding-msg:SRSSpatialInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'houseHoldId) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'houseHoldId)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWorkspaceOnMap-response>)))
  "Returns string type for a service object of type '<GetWorkspaceOnMap-response>"
  "srs_symbolic_grounding/GetWorkspaceOnMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWorkspaceOnMap-response)))
  "Returns string type for a service object of type 'GetWorkspaceOnMap-response"
  "srs_symbolic_grounding/GetWorkspaceOnMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWorkspaceOnMap-response>)))
  "Returns md5sum for a message object of type '<GetWorkspaceOnMap-response>"
  "4aac5a2e75ff4e1c2a3f288dcf79c97a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWorkspaceOnMap-response)))
  "Returns md5sum for a message object of type 'GetWorkspaceOnMap-response"
  "4aac5a2e75ff4e1c2a3f288dcf79c97a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWorkspaceOnMap-response>)))
  "Returns full string definition for message of type '<GetWorkspaceOnMap-response>"
  (cl:format cl:nil "~%~%~%~%~%string[] objects~%string[] classesOfObjects~%SRSSpatialInfo[] objectsInfo~%string[] houseHoldId~%~%================================================================================~%MSG: srs_symbolic_grounding/SRSSpatialInfo~%# Point point~%# Orientation angles~%float32 l~%float32 w~%float32 h~%~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWorkspaceOnMap-response)))
  "Returns full string definition for message of type 'GetWorkspaceOnMap-response"
  (cl:format cl:nil "~%~%~%~%~%string[] objects~%string[] classesOfObjects~%SRSSpatialInfo[] objectsInfo~%string[] houseHoldId~%~%================================================================================~%MSG: srs_symbolic_grounding/SRSSpatialInfo~%# Point point~%# Orientation angles~%float32 l~%float32 w~%float32 h~%~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWorkspaceOnMap-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'classesOfObjects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objectsInfo) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'houseHoldId) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWorkspaceOnMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWorkspaceOnMap-response
    (cl:cons ':objects (objects msg))
    (cl:cons ':classesOfObjects (classesOfObjects msg))
    (cl:cons ':objectsInfo (objectsInfo msg))
    (cl:cons ':houseHoldId (houseHoldId msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetWorkspaceOnMap)))
  'GetWorkspaceOnMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetWorkspaceOnMap)))
  'GetWorkspaceOnMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWorkspaceOnMap)))
  "Returns string type for a service object of type '<GetWorkspaceOnMap>"
  "srs_symbolic_grounding/GetWorkspaceOnMap")