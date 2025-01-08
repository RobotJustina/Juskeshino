; Auto-generated. Do not edit!


(cl:in-package robotino_msgs-srv)


;//! \htmlinclude SetEncoderPosition-request.msg.html

(cl:defclass <SetEncoderPosition-request> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type cl:integer
    :initform 0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:integer
    :initform 0))
)

(cl:defclass SetEncoderPosition-request (<SetEncoderPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetEncoderPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetEncoderPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotino_msgs-srv:<SetEncoderPosition-request> is deprecated: use robotino_msgs-srv:SetEncoderPosition-request instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <SetEncoderPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_msgs-srv:position-val is deprecated.  Use robotino_msgs-srv:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <SetEncoderPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_msgs-srv:velocity-val is deprecated.  Use robotino_msgs-srv:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetEncoderPosition-request>) ostream)
  "Serializes a message object of type '<SetEncoderPosition-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'position)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'position)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'position)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'velocity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'velocity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'velocity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'velocity)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetEncoderPosition-request>) istream)
  "Deserializes a message object of type '<SetEncoderPosition-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'position)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'position)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'position)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'velocity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'velocity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'velocity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'velocity)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetEncoderPosition-request>)))
  "Returns string type for a service object of type '<SetEncoderPosition-request>"
  "robotino_msgs/SetEncoderPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEncoderPosition-request)))
  "Returns string type for a service object of type 'SetEncoderPosition-request"
  "robotino_msgs/SetEncoderPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetEncoderPosition-request>)))
  "Returns md5sum for a message object of type '<SetEncoderPosition-request>"
  "e17f4b08b3e2cdbbf2b0d78e4f62e5d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetEncoderPosition-request)))
  "Returns md5sum for a message object of type 'SetEncoderPosition-request"
  "e17f4b08b3e2cdbbf2b0d78e4f62e5d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetEncoderPosition-request>)))
  "Returns full string definition for message of type '<SetEncoderPosition-request>"
  (cl:format cl:nil "uint32 position		# in encoder ticks~%uint32 velocity		# in ticks/s~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetEncoderPosition-request)))
  "Returns full string definition for message of type 'SetEncoderPosition-request"
  (cl:format cl:nil "uint32 position		# in encoder ticks~%uint32 velocity		# in ticks/s~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetEncoderPosition-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetEncoderPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetEncoderPosition-request
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
))
;//! \htmlinclude SetEncoderPosition-response.msg.html

(cl:defclass <SetEncoderPosition-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetEncoderPosition-response (<SetEncoderPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetEncoderPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetEncoderPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotino_msgs-srv:<SetEncoderPosition-response> is deprecated: use robotino_msgs-srv:SetEncoderPosition-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetEncoderPosition-response>) ostream)
  "Serializes a message object of type '<SetEncoderPosition-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetEncoderPosition-response>) istream)
  "Deserializes a message object of type '<SetEncoderPosition-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetEncoderPosition-response>)))
  "Returns string type for a service object of type '<SetEncoderPosition-response>"
  "robotino_msgs/SetEncoderPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEncoderPosition-response)))
  "Returns string type for a service object of type 'SetEncoderPosition-response"
  "robotino_msgs/SetEncoderPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetEncoderPosition-response>)))
  "Returns md5sum for a message object of type '<SetEncoderPosition-response>"
  "e17f4b08b3e2cdbbf2b0d78e4f62e5d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetEncoderPosition-response)))
  "Returns md5sum for a message object of type 'SetEncoderPosition-response"
  "e17f4b08b3e2cdbbf2b0d78e4f62e5d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetEncoderPosition-response>)))
  "Returns full string definition for message of type '<SetEncoderPosition-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetEncoderPosition-response)))
  "Returns full string definition for message of type 'SetEncoderPosition-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetEncoderPosition-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetEncoderPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetEncoderPosition-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetEncoderPosition)))
  'SetEncoderPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetEncoderPosition)))
  'SetEncoderPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEncoderPosition)))
  "Returns string type for a service object of type '<SetEncoderPosition>"
  "robotino_msgs/SetEncoderPosition")