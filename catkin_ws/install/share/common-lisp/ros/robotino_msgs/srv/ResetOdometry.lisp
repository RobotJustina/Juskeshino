; Auto-generated. Do not edit!


(cl:in-package robotino_msgs-srv)


;//! \htmlinclude ResetOdometry-request.msg.html

(cl:defclass <ResetOdometry-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (phi
    :reader phi
    :initarg :phi
    :type cl:float
    :initform 0.0))
)

(cl:defclass ResetOdometry-request (<ResetOdometry-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetOdometry-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetOdometry-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotino_msgs-srv:<ResetOdometry-request> is deprecated: use robotino_msgs-srv:ResetOdometry-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <ResetOdometry-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_msgs-srv:x-val is deprecated.  Use robotino_msgs-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <ResetOdometry-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_msgs-srv:y-val is deprecated.  Use robotino_msgs-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'phi-val :lambda-list '(m))
(cl:defmethod phi-val ((m <ResetOdometry-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_msgs-srv:phi-val is deprecated.  Use robotino_msgs-srv:phi instead.")
  (phi m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetOdometry-request>) ostream)
  "Serializes a message object of type '<ResetOdometry-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'phi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetOdometry-request>) istream)
  "Deserializes a message object of type '<ResetOdometry-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'phi) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetOdometry-request>)))
  "Returns string type for a service object of type '<ResetOdometry-request>"
  "robotino_msgs/ResetOdometryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetOdometry-request)))
  "Returns string type for a service object of type 'ResetOdometry-request"
  "robotino_msgs/ResetOdometryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetOdometry-request>)))
  "Returns md5sum for a message object of type '<ResetOdometry-request>"
  "400210218ff66e5bce9e5be16653c0c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetOdometry-request)))
  "Returns md5sum for a message object of type 'ResetOdometry-request"
  "400210218ff66e5bce9e5be16653c0c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetOdometry-request>)))
  "Returns full string definition for message of type '<ResetOdometry-request>"
  (cl:format cl:nil "float32 x # in meters~%float32 y # in meters~%float32 phi # in rad~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetOdometry-request)))
  "Returns full string definition for message of type 'ResetOdometry-request"
  (cl:format cl:nil "float32 x # in meters~%float32 y # in meters~%float32 phi # in rad~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetOdometry-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetOdometry-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetOdometry-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':phi (phi msg))
))
;//! \htmlinclude ResetOdometry-response.msg.html

(cl:defclass <ResetOdometry-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ResetOdometry-response (<ResetOdometry-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetOdometry-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetOdometry-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotino_msgs-srv:<ResetOdometry-response> is deprecated: use robotino_msgs-srv:ResetOdometry-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetOdometry-response>) ostream)
  "Serializes a message object of type '<ResetOdometry-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetOdometry-response>) istream)
  "Deserializes a message object of type '<ResetOdometry-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetOdometry-response>)))
  "Returns string type for a service object of type '<ResetOdometry-response>"
  "robotino_msgs/ResetOdometryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetOdometry-response)))
  "Returns string type for a service object of type 'ResetOdometry-response"
  "robotino_msgs/ResetOdometryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetOdometry-response>)))
  "Returns md5sum for a message object of type '<ResetOdometry-response>"
  "400210218ff66e5bce9e5be16653c0c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetOdometry-response)))
  "Returns md5sum for a message object of type 'ResetOdometry-response"
  "400210218ff66e5bce9e5be16653c0c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetOdometry-response>)))
  "Returns full string definition for message of type '<ResetOdometry-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetOdometry-response)))
  "Returns full string definition for message of type 'ResetOdometry-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetOdometry-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetOdometry-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetOdometry-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetOdometry)))
  'ResetOdometry-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetOdometry)))
  'ResetOdometry-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetOdometry)))
  "Returns string type for a service object of type '<ResetOdometry>"
  "robotino_msgs/ResetOdometry")