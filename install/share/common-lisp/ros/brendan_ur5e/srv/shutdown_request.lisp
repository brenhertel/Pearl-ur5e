; Auto-generated. Do not edit!


(cl:in-package brendan_ur5e-srv)


;//! \htmlinclude shutdown_request-request.msg.html

(cl:defclass <shutdown_request-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0))
)

(cl:defclass shutdown_request-request (<shutdown_request-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <shutdown_request-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'shutdown_request-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name brendan_ur5e-srv:<shutdown_request-request> is deprecated: use brendan_ur5e-srv:shutdown_request-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <shutdown_request-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brendan_ur5e-srv:a-val is deprecated.  Use brendan_ur5e-srv:a instead.")
  (a m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <shutdown_request-request>) ostream)
  "Serializes a message object of type '<shutdown_request-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <shutdown_request-request>) istream)
  "Deserializes a message object of type '<shutdown_request-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<shutdown_request-request>)))
  "Returns string type for a service object of type '<shutdown_request-request>"
  "brendan_ur5e/shutdown_requestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shutdown_request-request)))
  "Returns string type for a service object of type 'shutdown_request-request"
  "brendan_ur5e/shutdown_requestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<shutdown_request-request>)))
  "Returns md5sum for a message object of type '<shutdown_request-request>"
  "f16097f93022db785b2cc9436c158893")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'shutdown_request-request)))
  "Returns md5sum for a message object of type 'shutdown_request-request"
  "f16097f93022db785b2cc9436c158893")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<shutdown_request-request>)))
  "Returns full string definition for message of type '<shutdown_request-request>"
  (cl:format cl:nil "int64 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'shutdown_request-request)))
  "Returns full string definition for message of type 'shutdown_request-request"
  (cl:format cl:nil "int64 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <shutdown_request-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <shutdown_request-request>))
  "Converts a ROS message object to a list"
  (cl:list 'shutdown_request-request
    (cl:cons ':a (a msg))
))
;//! \htmlinclude shutdown_request-response.msg.html

(cl:defclass <shutdown_request-response> (roslisp-msg-protocol:ros-message)
  ((b
    :reader b
    :initarg :b
    :type cl:integer
    :initform 0))
)

(cl:defclass shutdown_request-response (<shutdown_request-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <shutdown_request-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'shutdown_request-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name brendan_ur5e-srv:<shutdown_request-response> is deprecated: use brendan_ur5e-srv:shutdown_request-response instead.")))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <shutdown_request-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader brendan_ur5e-srv:b-val is deprecated.  Use brendan_ur5e-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <shutdown_request-response>) ostream)
  "Serializes a message object of type '<shutdown_request-response>"
  (cl:let* ((signed (cl:slot-value msg 'b)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <shutdown_request-response>) istream)
  "Deserializes a message object of type '<shutdown_request-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<shutdown_request-response>)))
  "Returns string type for a service object of type '<shutdown_request-response>"
  "brendan_ur5e/shutdown_requestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shutdown_request-response)))
  "Returns string type for a service object of type 'shutdown_request-response"
  "brendan_ur5e/shutdown_requestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<shutdown_request-response>)))
  "Returns md5sum for a message object of type '<shutdown_request-response>"
  "f16097f93022db785b2cc9436c158893")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'shutdown_request-response)))
  "Returns md5sum for a message object of type 'shutdown_request-response"
  "f16097f93022db785b2cc9436c158893")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<shutdown_request-response>)))
  "Returns full string definition for message of type '<shutdown_request-response>"
  (cl:format cl:nil "int64 b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'shutdown_request-response)))
  "Returns full string definition for message of type 'shutdown_request-response"
  (cl:format cl:nil "int64 b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <shutdown_request-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <shutdown_request-response>))
  "Converts a ROS message object to a list"
  (cl:list 'shutdown_request-response
    (cl:cons ':b (b msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'shutdown_request)))
  'shutdown_request-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'shutdown_request)))
  'shutdown_request-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shutdown_request)))
  "Returns string type for a service object of type '<shutdown_request>"
  "brendan_ur5e/shutdown_request")