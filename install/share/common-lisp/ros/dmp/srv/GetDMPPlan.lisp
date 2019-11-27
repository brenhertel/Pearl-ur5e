; Auto-generated. Do not edit!


(cl:in-package dmp-srv)


;//! \htmlinclude GetDMPPlan-request.msg.html

(cl:defclass <GetDMPPlan-request> (roslisp-msg-protocol:ros-message)
  ((x_0
    :reader x_0
    :initarg :x_0
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (x_dot_0
    :reader x_dot_0
    :initarg :x_dot_0
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (t_0
    :reader t_0
    :initarg :t_0
    :type cl:float
    :initform 0.0)
   (goal
    :reader goal
    :initarg :goal
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (goal_thresh
    :reader goal_thresh
    :initarg :goal_thresh
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (seg_length
    :reader seg_length
    :initarg :seg_length
    :type cl:float
    :initform 0.0)
   (tau
    :reader tau
    :initarg :tau
    :type cl:float
    :initform 0.0)
   (dt
    :reader dt
    :initarg :dt
    :type cl:float
    :initform 0.0)
   (integrate_iter
    :reader integrate_iter
    :initarg :integrate_iter
    :type cl:integer
    :initform 0))
)

(cl:defclass GetDMPPlan-request (<GetDMPPlan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDMPPlan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDMPPlan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dmp-srv:<GetDMPPlan-request> is deprecated: use dmp-srv:GetDMPPlan-request instead.")))

(cl:ensure-generic-function 'x_0-val :lambda-list '(m))
(cl:defmethod x_0-val ((m <GetDMPPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:x_0-val is deprecated.  Use dmp-srv:x_0 instead.")
  (x_0 m))

(cl:ensure-generic-function 'x_dot_0-val :lambda-list '(m))
(cl:defmethod x_dot_0-val ((m <GetDMPPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:x_dot_0-val is deprecated.  Use dmp-srv:x_dot_0 instead.")
  (x_dot_0 m))

(cl:ensure-generic-function 't_0-val :lambda-list '(m))
(cl:defmethod t_0-val ((m <GetDMPPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:t_0-val is deprecated.  Use dmp-srv:t_0 instead.")
  (t_0 m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <GetDMPPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:goal-val is deprecated.  Use dmp-srv:goal instead.")
  (goal m))

(cl:ensure-generic-function 'goal_thresh-val :lambda-list '(m))
(cl:defmethod goal_thresh-val ((m <GetDMPPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:goal_thresh-val is deprecated.  Use dmp-srv:goal_thresh instead.")
  (goal_thresh m))

(cl:ensure-generic-function 'seg_length-val :lambda-list '(m))
(cl:defmethod seg_length-val ((m <GetDMPPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:seg_length-val is deprecated.  Use dmp-srv:seg_length instead.")
  (seg_length m))

(cl:ensure-generic-function 'tau-val :lambda-list '(m))
(cl:defmethod tau-val ((m <GetDMPPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:tau-val is deprecated.  Use dmp-srv:tau instead.")
  (tau m))

(cl:ensure-generic-function 'dt-val :lambda-list '(m))
(cl:defmethod dt-val ((m <GetDMPPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:dt-val is deprecated.  Use dmp-srv:dt instead.")
  (dt m))

(cl:ensure-generic-function 'integrate_iter-val :lambda-list '(m))
(cl:defmethod integrate_iter-val ((m <GetDMPPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:integrate_iter-val is deprecated.  Use dmp-srv:integrate_iter instead.")
  (integrate_iter m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDMPPlan-request>) ostream)
  "Serializes a message object of type '<GetDMPPlan-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'x_0))))
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
   (cl:slot-value msg 'x_0))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'x_dot_0))))
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
   (cl:slot-value msg 'x_dot_0))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 't_0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'goal))))
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
   (cl:slot-value msg 'goal))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'goal_thresh))))
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
   (cl:slot-value msg 'goal_thresh))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'seg_length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tau))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'integrate_iter)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDMPPlan-request>) istream)
  "Deserializes a message object of type '<GetDMPPlan-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'x_0) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'x_0)))
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
  (cl:setf (cl:slot-value msg 'x_dot_0) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'x_dot_0)))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't_0) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'goal) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'goal)))
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
  (cl:setf (cl:slot-value msg 'goal_thresh) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'goal_thresh)))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'seg_length) (roslisp-utils:decode-double-float-bits bits)))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dt) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'integrate_iter) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDMPPlan-request>)))
  "Returns string type for a service object of type '<GetDMPPlan-request>"
  "dmp/GetDMPPlanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDMPPlan-request)))
  "Returns string type for a service object of type 'GetDMPPlan-request"
  "dmp/GetDMPPlanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDMPPlan-request>)))
  "Returns md5sum for a message object of type '<GetDMPPlan-request>"
  "5cd79fd80676a4f8f062c5472a3190b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDMPPlan-request)))
  "Returns md5sum for a message object of type 'GetDMPPlan-request"
  "5cd79fd80676a4f8f062c5472a3190b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDMPPlan-request>)))
  "Returns full string definition for message of type '<GetDMPPlan-request>"
  (cl:format cl:nil "~%float64[] x_0~%~%~%float64[] x_dot_0~%~%~%~%float64 t_0~%~%~%float64[] goal~%~%~%~%~%~%float64[] goal_thresh~%~%~%float64 seg_length~%~%~%float64 tau~%~%~%float64 dt~%~%~%int32 integrate_iter~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDMPPlan-request)))
  "Returns full string definition for message of type 'GetDMPPlan-request"
  (cl:format cl:nil "~%float64[] x_0~%~%~%float64[] x_dot_0~%~%~%~%float64 t_0~%~%~%float64[] goal~%~%~%~%~%~%float64[] goal_thresh~%~%~%float64 seg_length~%~%~%float64 tau~%~%~%float64 dt~%~%~%int32 integrate_iter~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDMPPlan-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'x_0) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'x_dot_0) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'goal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'goal_thresh) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDMPPlan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDMPPlan-request
    (cl:cons ':x_0 (x_0 msg))
    (cl:cons ':x_dot_0 (x_dot_0 msg))
    (cl:cons ':t_0 (t_0 msg))
    (cl:cons ':goal (goal msg))
    (cl:cons ':goal_thresh (goal_thresh msg))
    (cl:cons ':seg_length (seg_length msg))
    (cl:cons ':tau (tau msg))
    (cl:cons ':dt (dt msg))
    (cl:cons ':integrate_iter (integrate_iter msg))
))
;//! \htmlinclude GetDMPPlan-response.msg.html

(cl:defclass <GetDMPPlan-response> (roslisp-msg-protocol:ros-message)
  ((plan
    :reader plan
    :initarg :plan
    :type dmp-msg:DMPTraj
    :initform (cl:make-instance 'dmp-msg:DMPTraj))
   (at_goal
    :reader at_goal
    :initarg :at_goal
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetDMPPlan-response (<GetDMPPlan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDMPPlan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDMPPlan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dmp-srv:<GetDMPPlan-response> is deprecated: use dmp-srv:GetDMPPlan-response instead.")))

(cl:ensure-generic-function 'plan-val :lambda-list '(m))
(cl:defmethod plan-val ((m <GetDMPPlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:plan-val is deprecated.  Use dmp-srv:plan instead.")
  (plan m))

(cl:ensure-generic-function 'at_goal-val :lambda-list '(m))
(cl:defmethod at_goal-val ((m <GetDMPPlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dmp-srv:at_goal-val is deprecated.  Use dmp-srv:at_goal instead.")
  (at_goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDMPPlan-response>) ostream)
  "Serializes a message object of type '<GetDMPPlan-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'plan) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'at_goal)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDMPPlan-response>) istream)
  "Deserializes a message object of type '<GetDMPPlan-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'plan) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'at_goal)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDMPPlan-response>)))
  "Returns string type for a service object of type '<GetDMPPlan-response>"
  "dmp/GetDMPPlanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDMPPlan-response)))
  "Returns string type for a service object of type 'GetDMPPlan-response"
  "dmp/GetDMPPlanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDMPPlan-response>)))
  "Returns md5sum for a message object of type '<GetDMPPlan-response>"
  "5cd79fd80676a4f8f062c5472a3190b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDMPPlan-response)))
  "Returns md5sum for a message object of type 'GetDMPPlan-response"
  "5cd79fd80676a4f8f062c5472a3190b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDMPPlan-response>)))
  "Returns full string definition for message of type '<GetDMPPlan-response>"
  (cl:format cl:nil "~%~%DMPTraj plan~%~%~%uint8 at_goal~%~%~%~%~%~%~%================================================================================~%MSG: dmp/DMPTraj~%# points and times should be the same length~%DMPPoint[] points~%~%# Times of observations, in seconds, starting at zero~%float64[] times~%~%~%~%================================================================================~%MSG: dmp/DMPPoint~%# Positions and velocities of DOFs~%#Velocity is only used for movement plans, not for giving demonstrations.~%float64[] positions~%float64[] velocities~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDMPPlan-response)))
  "Returns full string definition for message of type 'GetDMPPlan-response"
  (cl:format cl:nil "~%~%DMPTraj plan~%~%~%uint8 at_goal~%~%~%~%~%~%~%================================================================================~%MSG: dmp/DMPTraj~%# points and times should be the same length~%DMPPoint[] points~%~%# Times of observations, in seconds, starting at zero~%float64[] times~%~%~%~%================================================================================~%MSG: dmp/DMPPoint~%# Positions and velocities of DOFs~%#Velocity is only used for movement plans, not for giving demonstrations.~%float64[] positions~%float64[] velocities~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDMPPlan-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'plan))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDMPPlan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDMPPlan-response
    (cl:cons ':plan (plan msg))
    (cl:cons ':at_goal (at_goal msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDMPPlan)))
  'GetDMPPlan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDMPPlan)))
  'GetDMPPlan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDMPPlan)))
  "Returns string type for a service object of type '<GetDMPPlan>"
  "dmp/GetDMPPlan")