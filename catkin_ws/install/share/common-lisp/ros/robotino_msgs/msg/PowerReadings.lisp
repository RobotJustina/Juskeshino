; Auto-generated. Do not edit!


(cl:in-package robotino_msgs-msg)


;//! \htmlinclude PowerReadings.msg.html

(cl:defclass <PowerReadings> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0)
   (voltage
    :reader voltage
    :initarg :voltage
    :type cl:float
    :initform 0.0))
)

(cl:defclass PowerReadings (<PowerReadings>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PowerReadings>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PowerReadings)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotino_msgs-msg:<PowerReadings> is deprecated: use robotino_msgs-msg:PowerReadings instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <PowerReadings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_msgs-msg:stamp-val is deprecated.  Use robotino_msgs-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <PowerReadings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_msgs-msg:current-val is deprecated.  Use robotino_msgs-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <PowerReadings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_msgs-msg:voltage-val is deprecated.  Use robotino_msgs-msg:voltage instead.")
  (voltage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PowerReadings>) ostream)
  "Serializes a message object of type '<PowerReadings>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PowerReadings>) istream)
  "Deserializes a message object of type '<PowerReadings>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voltage) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PowerReadings>)))
  "Returns string type for a message object of type '<PowerReadings>"
  "robotino_msgs/PowerReadings")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PowerReadings)))
  "Returns string type for a message object of type 'PowerReadings"
  "robotino_msgs/PowerReadings")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PowerReadings>)))
  "Returns md5sum for a message object of type '<PowerReadings>"
  "a00aebd29e1775a226f3fa330a4fec59")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PowerReadings)))
  "Returns md5sum for a message object of type 'PowerReadings"
  "a00aebd29e1775a226f3fa330a4fec59")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PowerReadings>)))
  "Returns full string definition for message of type '<PowerReadings>"
  (cl:format cl:nil "time stamp	# time-stamp~%float32 current	# current in A~%float32 voltage	# voltage in V~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PowerReadings)))
  "Returns full string definition for message of type 'PowerReadings"
  (cl:format cl:nil "time stamp	# time-stamp~%float32 current	# current in A~%float32 voltage	# voltage in V~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PowerReadings>))
  (cl:+ 0
     8
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PowerReadings>))
  "Converts a ROS message object to a list"
  (cl:list 'PowerReadings
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':current (current msg))
    (cl:cons ':voltage (voltage msg))
))
