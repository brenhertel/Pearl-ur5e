; Auto-generated. Do not edit!


(cl:in-package dmp-srv)


;//! \htmlinclude SetActiveDMP-request.msg.html

(cl:defclass <SetActiveDMP-request> (roslisp-msg-protocol:ros-message)
  ((dmp_list
    :reader dmp_list
    :initarg :dmp_list
    :type (cl:vector dmp-msg:DMPData)
   :initform (cl:make-array 0 :element-type 'dmp-msg:DMPData :initial-element (cl:make-instance 'dmp-msg:DMPData))))
)

(cl:defclass SetActiveDMP-request (<SetActiveDMP-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetActiveDMP-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetActiveDMP-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dmp-srv:<SetActiveDMP-request> is deprecated: use dmp-srv:SetActiveDMP-request instead.")))

(cl:ensure-generic-function 'dmp_list-val :lambda-list '(m))
(cl:defmethod dmp_list-val ((m <SetActiveDMP-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:dmp_list-val is deprecated.  Use dmp-srv:dmp_list instead.")
  (dmp_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetActiveDMP-request>) ostream)
  "Serializes a message object of type '<SetActiveDMP-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'dmp_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'dmp_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetActiveDMP-request>) istream)
  "Deserializes a message object of type '<SetActiveDMP-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'dmp_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'dmp_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'dmp-msg:DMPData))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetActiveDMP-request>)))
  "Returns string type for a service object of type '<SetActiveDMP-request>"
  "dmp/SetActiveDMPRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetActiveDMP-request)))
  "Returns string type for a service object of type 'SetActiveDMP-request"
  "dmp/SetActiveDMPRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetActiveDMP-request>)))
  "Returns md5sum for a message object of type '<SetActiveDMP-request>"
  "10d64adb0a08cbb7afbd425801f828e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetActiveDMP-request)))
  "Returns md5sum for a message object of type 'SetActiveDMP-request"
  "10d64adb0a08cbb7afbd425801f828e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetActiveDMP-request>)))
  "Returns full string definition for message of type '<SetActiveDMP-request>"
  (cl:format cl:nil "~%DMPData[] dmp_list~%~%~%================================================================================~%MSG: dmp/DMPData~%float64 k_gain~%float64 d_gain~%float64[] weights~%float64[] f_domain~%float64[] f_targets~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetActiveDMP-request)))
  "Returns full string definition for message of type 'SetActiveDMP-request"
  (cl:format cl:nil "~%DMPData[] dmp_list~%~%~%================================================================================~%MSG: dmp/DMPData~%float64 k_gain~%float64 d_gain~%float64[] weights~%float64[] f_domain~%float64[] f_targets~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetActiveDMP-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'dmp_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetActiveDMP-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetActiveDMP-request
    (cl:cons ':dmp_list (dmp_list msg))
))
;//! \htmlinclude SetActiveDMP-response.msg.html

(cl:defclass <SetActiveDMP-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetActiveDMP-response (<SetActiveDMP-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetActiveDMP-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetActiveDMP-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dmp-srv:<SetActiveDMP-response> is deprecated: use dmp-srv:SetActiveDMP-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetActiveDMP-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:success-val is deprecated.  Use dmp-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetActiveDMP-response>) ostream)
  "Serializes a message object of type '<SetActiveDMP-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetActiveDMP-response>) istream)
  "Deserializes a message object of type '<SetActiveDMP-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetActiveDMP-response>)))
  "Returns string type for a service object of type '<SetActiveDMP-response>"
  "dmp/SetActiveDMPResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetActiveDMP-response)))
  "Returns string type for a service object of type 'SetActiveDMP-response"
  "dmp/SetActiveDMPResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetActiveDMP-response>)))
  "Returns md5sum for a message object of type '<SetActiveDMP-response>"
  "10d64adb0a08cbb7afbd425801f828e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetActiveDMP-response)))
  "Returns md5sum for a message object of type 'SetActiveDMP-response"
  "10d64adb0a08cbb7afbd425801f828e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetActiveDMP-response>)))
  "Returns full string definition for message of type '<SetActiveDMP-response>"
  (cl:format cl:nil "~%~%bool success~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetActiveDMP-response)))
  "Returns full string definition for message of type 'SetActiveDMP-response"
  (cl:format cl:nil "~%~%bool success~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetActiveDMP-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetActiveDMP-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetActiveDMP-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetActiveDMP)))
  'SetActiveDMP-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetActiveDMP)))
  'SetActiveDMP-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetActiveDMP)))
  "Returns string type for a service object of type '<SetActiveDMP>"
  "dmp/SetActiveDMP")