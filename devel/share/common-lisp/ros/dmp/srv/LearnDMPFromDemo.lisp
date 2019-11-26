; Auto-generated. Do not edit!


(cl:in-package dmp-srv)


;//! \htmlinclude LearnDMPFromDemo-request.msg.html

(cl:defclass <LearnDMPFromDemo-request> (roslisp-msg-protocol:ros-message)
  ((demo
    :reader demo
    :initarg :demo
    :type dmp-msg:DMPTraj
    :initform (cl:make-instance 'dmp-msg:DMPTraj))
   (k_gains
    :reader k_gains
    :initarg :k_gains
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (d_gains
    :reader d_gains
    :initarg :d_gains
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (num_bases
    :reader num_bases
    :initarg :num_bases
    :type cl:integer
    :initform 0))
)

(cl:defclass LearnDMPFromDemo-request (<LearnDMPFromDemo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LearnDMPFromDemo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LearnDMPFromDemo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dmp-srv:<LearnDMPFromDemo-request> is deprecated: use dmp-srv:LearnDMPFromDemo-request instead.")))

(cl:ensure-generic-function 'demo-val :lambda-list '(m))
(cl:defmethod demo-val ((m <LearnDMPFromDemo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:demo-val is deprecated.  Use dmp-srv:demo instead.")
  (demo m))

(cl:ensure-generic-function 'k_gains-val :lambda-list '(m))
(cl:defmethod k_gains-val ((m <LearnDMPFromDemo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:k_gains-val is deprecated.  Use dmp-srv:k_gains instead.")
  (k_gains m))

(cl:ensure-generic-function 'd_gains-val :lambda-list '(m))
(cl:defmethod d_gains-val ((m <LearnDMPFromDemo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:d_gains-val is deprecated.  Use dmp-srv:d_gains instead.")
  (d_gains m))

(cl:ensure-generic-function 'num_bases-val :lambda-list '(m))
(cl:defmethod num_bases-val ((m <LearnDMPFromDemo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:num_bases-val is deprecated.  Use dmp-srv:num_bases instead.")
  (num_bases m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LearnDMPFromDemo-request>) ostream)
  "Serializes a message object of type '<LearnDMPFromDemo-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'demo) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'k_gains))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'k_gains))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'd_gains))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'd_gains))
  (cl:let* ((signed (cl:slot-value msg 'num_bases)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LearnDMPFromDemo-request>) istream)
  "Deserializes a message object of type '<LearnDMPFromDemo-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'demo) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'k_gains) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'k_gains)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'd_gains) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'd_gains)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_bases) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LearnDMPFromDemo-request>)))
  "Returns string type for a service object of type '<LearnDMPFromDemo-request>"
  "dmp/LearnDMPFromDemoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LearnDMPFromDemo-request)))
  "Returns string type for a service object of type 'LearnDMPFromDemo-request"
  "dmp/LearnDMPFromDemoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LearnDMPFromDemo-request>)))
  "Returns md5sum for a message object of type '<LearnDMPFromDemo-request>"
  "3ba13cfa47585560a2fd9cc202efdbff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LearnDMPFromDemo-request)))
  "Returns md5sum for a message object of type 'LearnDMPFromDemo-request"
  "3ba13cfa47585560a2fd9cc202efdbff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LearnDMPFromDemo-request>)))
  "Returns full string definition for message of type '<LearnDMPFromDemo-request>"
  (cl:format cl:nil "~%DMPTraj demo~%~%~%float64[] k_gains~%float64[] d_gains~%~%~%int32 num_bases~%~%~%================================================================================~%MSG: dmp/DMPTraj~%# points and times should be the same length~%DMPPoint[] points~%~%# Times of observations, in seconds, starting at zero~%float64[] times~%~%~%~%================================================================================~%MSG: dmp/DMPPoint~%# Positions and velocities of DOFs~%#Velocity is only used for movement plans, not for giving demonstrations.~%float64[] positions~%float64[] velocities~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LearnDMPFromDemo-request)))
  "Returns full string definition for message of type 'LearnDMPFromDemo-request"
  (cl:format cl:nil "~%DMPTraj demo~%~%~%float64[] k_gains~%float64[] d_gains~%~%~%int32 num_bases~%~%~%================================================================================~%MSG: dmp/DMPTraj~%# points and times should be the same length~%DMPPoint[] points~%~%# Times of observations, in seconds, starting at zero~%float64[] times~%~%~%~%================================================================================~%MSG: dmp/DMPPoint~%# Positions and velocities of DOFs~%#Velocity is only used for movement plans, not for giving demonstrations.~%float64[] positions~%float64[] velocities~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LearnDMPFromDemo-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'demo))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'k_gains) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'd_gains) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LearnDMPFromDemo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LearnDMPFromDemo-request
    (cl:cons ':demo (demo msg))
    (cl:cons ':k_gains (k_gains msg))
    (cl:cons ':d_gains (d_gains msg))
    (cl:cons ':num_bases (num_bases msg))
))
;//! \htmlinclude LearnDMPFromDemo-response.msg.html

(cl:defclass <LearnDMPFromDemo-response> (roslisp-msg-protocol:ros-message)
  ((dmp_list
    :reader dmp_list
    :initarg :dmp_list
    :type (cl:vector dmp-msg:DMPData)
   :initform (cl:make-array 0 :element-type 'dmp-msg:DMPData :initial-element (cl:make-instance 'dmp-msg:DMPData)))
   (tau
    :reader tau
    :initarg :tau
    :type cl:float
    :initform 0.0))
)

(cl:defclass LearnDMPFromDemo-response (<LearnDMPFromDemo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LearnDMPFromDemo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LearnDMPFromDemo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dmp-srv:<LearnDMPFromDemo-response> is deprecated: use dmp-srv:LearnDMPFromDemo-response instead.")))

(cl:ensure-generic-function 'dmp_list-val :lambda-list '(m))
(cl:defmethod dmp_list-val ((m <LearnDMPFromDemo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:dmp_list-val is deprecated.  Use dmp-srv:dmp_list instead.")
  (dmp_list m))

(cl:ensure-generic-function 'tau-val :lambda-list '(m))
(cl:defmethod tau-val ((m <LearnDMPFromDemo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:tau-val is deprecated.  Use dmp-srv:tau instead.")
  (tau m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LearnDMPFromDemo-response>) ostream)
  "Serializes a message object of type '<LearnDMPFromDemo-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'dmp_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'dmp_list))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tau))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LearnDMPFromDemo-response>) istream)
  "Deserializes a message object of type '<LearnDMPFromDemo-response>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tau) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LearnDMPFromDemo-response>)))
  "Returns string type for a service object of type '<LearnDMPFromDemo-response>"
  "dmp/LearnDMPFromDemoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LearnDMPFromDemo-response)))
  "Returns string type for a service object of type 'LearnDMPFromDemo-response"
  "dmp/LearnDMPFromDemoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LearnDMPFromDemo-response>)))
  "Returns md5sum for a message object of type '<LearnDMPFromDemo-response>"
  "3ba13cfa47585560a2fd9cc202efdbff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LearnDMPFromDemo-response)))
  "Returns md5sum for a message object of type 'LearnDMPFromDemo-response"
  "3ba13cfa47585560a2fd9cc202efdbff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LearnDMPFromDemo-response>)))
  "Returns full string definition for message of type '<LearnDMPFromDemo-response>"
  (cl:format cl:nil "~%~%DMPData[] dmp_list~%~%~%float64 tau~%~%~%================================================================================~%MSG: dmp/DMPData~%float64 k_gain~%float64 d_gain~%float64[] weights~%float64[] f_domain~%float64[] f_targets~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LearnDMPFromDemo-response)))
  "Returns full string definition for message of type 'LearnDMPFromDemo-response"
  (cl:format cl:nil "~%~%DMPData[] dmp_list~%~%~%float64 tau~%~%~%================================================================================~%MSG: dmp/DMPData~%float64 k_gain~%float64 d_gain~%float64[] weights~%float64[] f_domain~%float64[] f_targets~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LearnDMPFromDemo-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'dmp_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LearnDMPFromDemo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LearnDMPFromDemo-response
    (cl:cons ':dmp_list (dmp_list msg))
    (cl:cons ':tau (tau msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LearnDMPFromDemo)))
  'LearnDMPFromDemo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LearnDMPFromDemo)))
  'LearnDMPFromDemo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LearnDMPFromDemo)))
  "Returns string type for a service object of type '<LearnDMPFromDemo>"
  "dmp/LearnDMPFromDemo")