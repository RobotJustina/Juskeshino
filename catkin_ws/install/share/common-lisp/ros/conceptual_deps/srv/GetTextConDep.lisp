; Auto-generated. Do not edit!


(cl:in-package conceptual_deps-srv)


;//! \htmlinclude GetTextConDep-request.msg.html

(cl:defclass <GetTextConDep-request> (roslisp-msg-protocol:ros-message)
  ((text
    :reader text
    :initarg :text
    :type cl:string
    :initform ""))
)

(cl:defclass GetTextConDep-request (<GetTextConDep-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTextConDep-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTextConDep-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name conceptual_deps-srv:<GetTextConDep-request> is deprecated: use conceptual_deps-srv:GetTextConDep-request instead.")))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <GetTextConDep-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader conceptual_deps-srv:text-val is deprecated.  Use conceptual_deps-srv:text instead.")
  (text m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTextConDep-request>) ostream)
  "Serializes a message object of type '<GetTextConDep-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTextConDep-request>) istream)
  "Deserializes a message object of type '<GetTextConDep-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTextConDep-request>)))
  "Returns string type for a service object of type '<GetTextConDep-request>"
  "conceptual_deps/GetTextConDepRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTextConDep-request)))
  "Returns string type for a service object of type 'GetTextConDep-request"
  "conceptual_deps/GetTextConDepRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTextConDep-request>)))
  "Returns md5sum for a message object of type '<GetTextConDep-request>"
  "b7e9f26875b56c920fd22b0d43671c0b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTextConDep-request)))
  "Returns md5sum for a message object of type 'GetTextConDep-request"
  "b7e9f26875b56c920fd22b0d43671c0b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTextConDep-request>)))
  "Returns full string definition for message of type '<GetTextConDep-request>"
  (cl:format cl:nil "string text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTextConDep-request)))
  "Returns full string definition for message of type 'GetTextConDep-request"
  (cl:format cl:nil "string text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTextConDep-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'text))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTextConDep-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTextConDep-request
    (cl:cons ':text (text msg))
))
;//! \htmlinclude GetTextConDep-response.msg.html

(cl:defclass <GetTextConDep-response> (roslisp-msg-protocol:ros-message)
  ((cds
    :reader cds
    :initarg :cds
    :type conceptual_deps-msg:StringArray
    :initform (cl:make-instance 'conceptual_deps-msg:StringArray)))
)

(cl:defclass GetTextConDep-response (<GetTextConDep-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTextConDep-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTextConDep-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name conceptual_deps-srv:<GetTextConDep-response> is deprecated: use conceptual_deps-srv:GetTextConDep-response instead.")))

(cl:ensure-generic-function 'cds-val :lambda-list '(m))
(cl:defmethod cds-val ((m <GetTextConDep-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader conceptual_deps-srv:cds-val is deprecated.  Use conceptual_deps-srv:cds instead.")
  (cds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTextConDep-response>) ostream)
  "Serializes a message object of type '<GetTextConDep-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cds) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTextConDep-response>) istream)
  "Deserializes a message object of type '<GetTextConDep-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cds) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTextConDep-response>)))
  "Returns string type for a service object of type '<GetTextConDep-response>"
  "conceptual_deps/GetTextConDepResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTextConDep-response)))
  "Returns string type for a service object of type 'GetTextConDep-response"
  "conceptual_deps/GetTextConDepResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTextConDep-response>)))
  "Returns md5sum for a message object of type '<GetTextConDep-response>"
  "b7e9f26875b56c920fd22b0d43671c0b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTextConDep-response)))
  "Returns md5sum for a message object of type 'GetTextConDep-response"
  "b7e9f26875b56c920fd22b0d43671c0b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTextConDep-response>)))
  "Returns full string definition for message of type '<GetTextConDep-response>"
  (cl:format cl:nil "StringArray  cds~%~%~%================================================================================~%MSG: conceptual_deps/StringArray~%string[]  data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTextConDep-response)))
  "Returns full string definition for message of type 'GetTextConDep-response"
  (cl:format cl:nil "StringArray  cds~%~%~%================================================================================~%MSG: conceptual_deps/StringArray~%string[]  data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTextConDep-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cds))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTextConDep-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTextConDep-response
    (cl:cons ':cds (cds msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTextConDep)))
  'GetTextConDep-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTextConDep)))
  'GetTextConDep-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTextConDep)))
  "Returns string type for a service object of type '<GetTextConDep>"
  "conceptual_deps/GetTextConDep")