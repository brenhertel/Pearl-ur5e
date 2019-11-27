; Auto-generated. Do not edit!


(cl:in-package dmp-msg)


;//! \htmlinclude DMPTraj.msg.html

(cl:defclass <DMPTraj> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector dmp-msg:DMPPoint)
   :initform (cl:make-array 0 :element-type 'dmp-msg:DMPPoint :initial-element (cl:make-instance 'dmp-msg:DMPPoint)))
   (times
    :reader times
    :initarg :times
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass DMPTraj (<DMPTraj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DMPTraj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DMPTraj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dmp-msg:<DMPTraj> is deprecated: use dmp-msg:DMPTraj instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <DMPTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-msg:points-val is deprecated.  Use dmp-msg:points instead.")
  (points m))

(cl:ensure-generic-function 'times-val :lambda-list '(m))
(cl:defmethod times-val ((m <DMPTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-msg:times-val is deprecated.  Use dmp-msg:times instead.")
  (times m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DMPTraj>) ostream)
  "Serializes a message object of type '<DMPTraj>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'times))))
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
   (cl:slot-value msg 'times))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DMPTraj>) istream)
  "Deserializes a message object of type '<DMPTraj>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'dmp-msg:DMPPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'times) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'times)))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DMPTraj>)))
  "Returns string type for a message object of type '<DMPTraj>"
  "dmp/DMPTraj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DMPTraj)))
  "Returns string type for a message object of type 'DMPTraj"
  "dmp/DMPTraj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DMPTraj>)))
  "Returns md5sum for a message object of type '<DMPTraj>"
  "1d088d86ab60cf6a2671bc3c0e99932b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DMPTraj)))
  "Returns md5sum for a message object of type 'DMPTraj"
  "1d088d86ab60cf6a2671bc3c0e99932b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DMPTraj>)))
  "Returns full string definition for message of type '<DMPTraj>"
  (cl:format cl:nil "# points and times should be the same length~%DMPPoint[] points~%~%# Times of observations, in seconds, starting at zero~%float64[] times~%~%~%~%================================================================================~%MSG: dmp/DMPPoint~%# Positions and velocities of DOFs~%#Velocity is only used for movement plans, not for giving demonstrations.~%float64[] positions~%float64[] velocities~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DMPTraj)))
  "Returns full string definition for message of type 'DMPTraj"
  (cl:format cl:nil "# points and times should be the same length~%DMPPoint[] points~%~%# Times of observations, in seconds, starting at zero~%float64[] times~%~%~%~%================================================================================~%MSG: dmp/DMPPoint~%# Positions and velocities of DOFs~%#Velocity is only used for movement plans, not for giving demonstrations.~%float64[] positions~%float64[] velocities~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DMPTraj>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'times) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DMPTraj>))
  "Converts a ROS message object to a list"
  (cl:list 'DMPTraj
    (cl:cons ':points (points msg))
    (cl:cons ':times (times msg))
))
