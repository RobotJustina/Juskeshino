; Auto-generated. Do not edit!


(cl:in-package robotino_msgs-msg)


;//! \htmlinclude EncoderReadings.msg.html

(cl:defclass <EncoderReadings> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:integer
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type cl:integer
    :initform 0)
   (current
    :reader current
    :initarg :current
    :type cl:integer
    :initform 0))
)

(cl:defclass EncoderReadings (<EncoderReadings>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EncoderReadings>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EncoderReadings)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotino_msgs-msg:<EncoderReadings> is deprecated: use robotino_msgs-msg:EncoderReadings instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <EncoderReadings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_msgs-msg:stamp-val is deprecated.  Use robotino_msgs-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <EncoderReadings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_msgs-msg:velocity-val is deprecated.  Use robotino_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <EncoderReadings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_msgs-msg:position-val is deprecated.  Use robotino_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <EncoderReadings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_msgs-msg:current-val is deprecated.  Use robotino_msgs-msg:current instead.")
  (current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EncoderReadings>) ostream)
  "Serializes a message object of type '<EncoderReadings>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'velocity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'velocity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'velocity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'velocity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'position)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'position)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'position)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'current)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'current)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'current)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'current)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EncoderReadings>) istream)
  "Deserializes a message object of type '<EncoderReadings>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'velocity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'velocity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'velocity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'velocity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'position)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'position)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'position)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'current)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'current)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'current)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'current)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EncoderReadings>)))
  "Returns string type for a message object of type '<EncoderReadings>"
  "robotino_msgs/EncoderReadings")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EncoderReadings)))
  "Returns string type for a message object of type 'EncoderReadings"
  "robotino_msgs/EncoderReadings")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EncoderReadings>)))
  "Returns md5sum for a message object of type '<EncoderReadings>"
  "0b4033dda61bb04d3e2ea6c671f26183")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EncoderReadings)))
  "Returns md5sum for a message object of type 'EncoderReadings"
  "0b4033dda61bb04d3e2ea6c671f26183")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EncoderReadings>)))
  "Returns full string definition for message of type '<EncoderReadings>"
  (cl:format cl:nil "time stamp~%uint32 velocity		# actual velocity in ticks/s~%uint32 position		# actual position in ticks~%uint32 current		# in A~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EncoderReadings)))
  "Returns full string definition for message of type 'EncoderReadings"
  (cl:format cl:nil "time stamp~%uint32 velocity		# actual velocity in ticks/s~%uint32 position		# actual position in ticks~%uint32 current		# in A~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EncoderReadings>))
  (cl:+ 0
     8
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EncoderReadings>))
  "Converts a ROS message object to a list"
  (cl:list 'EncoderReadings
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':position (position msg))
    (cl:cons ':current (current msg))
))
