; Auto-generated. Do not edit!


(cl:in-package conceptual_deps-srv)


;//! \htmlinclude GetConDep-request.msg.html

(cl:defclass <GetConDep-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetConDep-request (<GetConDep-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetConDep-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetConDep-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name conceptual_deps-srv:<GetConDep-request> is deprecated: use conceptual_deps-srv:GetConDep-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetConDep-request>) ostream)
  "Serializes a message object of type '<GetConDep-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetConDep-request>) istream)
  "Deserializes a message object of type '<GetConDep-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetConDep-request>)))
  "Returns string type for a service object of type '<GetConDep-request>"
  "conceptual_deps/GetConDepRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConDep-request)))
  "Returns string type for a service object of type 'GetConDep-request"
  "conceptual_deps/GetConDepRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetConDep-request>)))
  "Returns md5sum for a message object of type '<GetConDep-request>"
  "6530474bcb00416881ea95591006435a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetConDep-request)))
  "Returns md5sum for a message object of type 'GetConDep-request"
  "6530474bcb00416881ea95591006435a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetConDep-request>)))
  "Returns full string definition for message of type '<GetConDep-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetConDep-request)))
  "Returns full string definition for message of type 'GetConDep-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetConDep-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetConDep-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetConDep-request
))
;//! \htmlinclude GetConDep-response.msg.html

(cl:defclass <GetConDep-response> (roslisp-msg-protocol:ros-message)
  ((cds
    :reader cds
    :initarg :cds
    :type conceptual_deps-msg:StringArray
    :initform (cl:make-instance 'conceptual_deps-msg:StringArray)))
)

(cl:defclass GetConDep-response (<GetConDep-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetConDep-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetConDep-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name conceptual_deps-srv:<GetConDep-response> is deprecated: use conceptual_deps-srv:GetConDep-response instead.")))

(cl:ensure-generic-function 'cds-val :lambda-list '(m))
(cl:defmethod cds-val ((m <GetConDep-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader conceptual_deps-srv:cds-val is deprecated.  Use conceptual_deps-srv:cds instead.")
  (cds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetConDep-response>) ostream)
  "Serializes a message object of type '<GetConDep-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cds) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetConDep-response>) istream)
  "Deserializes a message object of type '<GetConDep-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cds) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetConDep-response>)))
  "Returns string type for a service object of type '<GetConDep-response>"
  "conceptual_deps/GetConDepResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConDep-response)))
  "Returns string type for a service object of type 'GetConDep-response"
  "conceptual_deps/GetConDepResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetConDep-response>)))
  "Returns md5sum for a message object of type '<GetConDep-response>"
  "6530474bcb00416881ea95591006435a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetConDep-response)))
  "Returns md5sum for a message object of type 'GetConDep-response"
  "6530474bcb00416881ea95591006435a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetConDep-response>)))
  "Returns full string definition for message of type '<GetConDep-response>"
  (cl:format cl:nil "StringArray  cds~%~%~%================================================================================~%MSG: conceptual_deps/StringArray~%string[]  data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetConDep-response)))
  "Returns full string definition for message of type 'GetConDep-response"
  (cl:format cl:nil "StringArray  cds~%~%~%================================================================================~%MSG: conceptual_deps/StringArray~%string[]  data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetConDep-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cds))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetConDep-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetConDep-response
    (cl:cons ':cds (cds msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetConDep)))
  'GetConDep-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetConDep)))
  'GetConDep-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConDep)))
  "Returns string type for a service object of type '<GetConDep>"
  "conceptual_deps/GetConDep")