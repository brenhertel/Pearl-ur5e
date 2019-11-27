;; Auto-generated. Do not edit!


(when (boundp 'dmp::GetDMPPlan)
  (if (not (find-package "DMP"))
    (make-package "DMP"))
  (shadow 'GetDMPPlan (find-package "DMP")))
(unless (find-package "DMP::GETDMPPLAN")
  (make-package "DMP::GETDMPPLAN"))
(unless (find-package "DMP::GETDMPPLANREQUEST")
  (make-package "DMP::GETDMPPLANREQUEST"))
(unless (find-package "DMP::GETDMPPLANRESPONSE")
  (make-package "DMP::GETDMPPLANRESPONSE"))

(in-package "ROS")





(defclass dmp::GetDMPPlanRequest
  :super ros::object
  :slots (_x_0 _x_dot_0 _t_0 _goal _goal_thresh _seg_length _tau _dt _integrate_iter ))

(defmethod dmp::GetDMPPlanRequest
  (:init
   (&key
    ((:x_0 __x_0) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:x_dot_0 __x_dot_0) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:t_0 __t_0) 0.0)
    ((:goal __goal) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:goal_thresh __goal_thresh) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:seg_length __seg_length) 0.0)
    ((:tau __tau) 0.0)
    ((:dt __dt) 0.0)
    ((:integrate_iter __integrate_iter) 0)
    )
   (send-super :init)
   (setq _x_0 __x_0)
   (setq _x_dot_0 __x_dot_0)
   (setq _t_0 (float __t_0))
   (setq _goal __goal)
   (setq _goal_thresh __goal_thresh)
   (setq _seg_length (float __seg_length))
   (setq _tau (float __tau))
   (setq _dt (float __dt))
   (setq _integrate_iter (round __integrate_iter))
   self)
  (:x_0
   (&optional __x_0)
   (if __x_0 (setq _x_0 __x_0)) _x_0)
  (:x_dot_0
   (&optional __x_dot_0)
   (if __x_dot_0 (setq _x_dot_0 __x_dot_0)) _x_dot_0)
  (:t_0
   (&optional __t_0)
   (if __t_0 (setq _t_0 __t_0)) _t_0)
  (:goal
   (&optional __goal)
   (if __goal (setq _goal __goal)) _goal)
  (:goal_thresh
   (&optional __goal_thresh)
   (if __goal_thresh (setq _goal_thresh __goal_thresh)) _goal_thresh)
  (:seg_length
   (&optional __seg_length)
   (if __seg_length (setq _seg_length __seg_length)) _seg_length)
  (:tau
   (&optional __tau)
   (if __tau (setq _tau __tau)) _tau)
  (:dt
   (&optional __dt)
   (if __dt (setq _dt __dt)) _dt)
  (:integrate_iter
   (&optional __integrate_iter)
   (if __integrate_iter (setq _integrate_iter __integrate_iter)) _integrate_iter)
  (:serialization-length
   ()
   (+
    ;; float64[] _x_0
    (* 8    (length _x_0)) 4
    ;; float64[] _x_dot_0
    (* 8    (length _x_dot_0)) 4
    ;; float64 _t_0
    8
    ;; float64[] _goal
    (* 8    (length _goal)) 4
    ;; float64[] _goal_thresh
    (* 8    (length _goal_thresh)) 4
    ;; float64 _seg_length
    8
    ;; float64 _tau
    8
    ;; float64 _dt
    8
    ;; int32 _integrate_iter
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _x_0
     (write-long (length _x_0) s)
     (dotimes (i (length _x_0))
       (sys::poke (elt _x_0 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _x_dot_0
     (write-long (length _x_dot_0) s)
     (dotimes (i (length _x_dot_0))
       (sys::poke (elt _x_dot_0 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _t_0
       (sys::poke _t_0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _goal
     (write-long (length _goal) s)
     (dotimes (i (length _goal))
       (sys::poke (elt _goal i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _goal_thresh
     (write-long (length _goal_thresh) s)
     (dotimes (i (length _goal_thresh))
       (sys::poke (elt _goal_thresh i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _seg_length
       (sys::poke _seg_length (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _tau
       (sys::poke _tau (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dt
       (sys::poke _dt (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _integrate_iter
       (write-long _integrate_iter s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _x_0
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x_0 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x_0 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _x_dot_0
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x_dot_0 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x_dot_0 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _t_0
     (setq _t_0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _goal
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _goal (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _goal i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _goal_thresh
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _goal_thresh (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _goal_thresh i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _seg_length
     (setq _seg_length (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _tau
     (setq _tau (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dt
     (setq _dt (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _integrate_iter
     (setq _integrate_iter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dmp::GetDMPPlanResponse
  :super ros::object
  :slots (_plan _at_goal ))

(defmethod dmp::GetDMPPlanResponse
  (:init
   (&key
    ((:plan __plan) (instance dmp::DMPTraj :init))
    ((:at_goal __at_goal) 0)
    )
   (send-super :init)
   (setq _plan __plan)
   (setq _at_goal (round __at_goal))
   self)
  (:plan
   (&rest __plan)
   (if (keywordp (car __plan))
       (send* _plan __plan)
     (progn
       (if __plan (setq _plan (car __plan)))
       _plan)))
  (:at_goal
   (&optional __at_goal)
   (if __at_goal (setq _at_goal __at_goal)) _at_goal)
  (:serialization-length
   ()
   (+
    ;; dmp/DMPTraj _plan
    (send _plan :serialization-length)
    ;; uint8 _at_goal
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; dmp/DMPTraj _plan
       (send _plan :serialize s)
     ;; uint8 _at_goal
       (write-byte _at_goal s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; dmp/DMPTraj _plan
     (send _plan :deserialize buf ptr-) (incf ptr- (send _plan :serialization-length))
   ;; uint8 _at_goal
     (setq _at_goal (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass dmp::GetDMPPlan
  :super ros::object
  :slots ())

(setf (get dmp::GetDMPPlan :md5sum-) "5cd79fd80676a4f8f062c5472a3190b1")
(setf (get dmp::GetDMPPlan :datatype-) "dmp/GetDMPPlan")
(setf (get dmp::GetDMPPlan :request) dmp::GetDMPPlanRequest)
(setf (get dmp::GetDMPPlan :response) dmp::GetDMPPlanResponse)

(defmethod dmp::GetDMPPlanRequest
  (:response () (instance dmp::GetDMPPlanResponse :init)))

(setf (get dmp::GetDMPPlanRequest :md5sum-) "5cd79fd80676a4f8f062c5472a3190b1")
(setf (get dmp::GetDMPPlanRequest :datatype-) "dmp/GetDMPPlanRequest")
(setf (get dmp::GetDMPPlanRequest :definition-)
      "
float64[] x_0


float64[] x_dot_0



float64 t_0


float64[] goal





float64[] goal_thresh


float64 seg_length


float64 tau


float64 dt


int32 integrate_iter

---


DMPTraj plan


uint8 at_goal






================================================================================
MSG: dmp/DMPTraj
# points and times should be the same length
DMPPoint[] points

# Times of observations, in seconds, starting at zero
float64[] times



================================================================================
MSG: dmp/DMPPoint
# Positions and velocities of DOFs
#Velocity is only used for movement plans, not for giving demonstrations.
float64[] positions
float64[] velocities


")

(setf (get dmp::GetDMPPlanResponse :md5sum-) "5cd79fd80676a4f8f062c5472a3190b1")
(setf (get dmp::GetDMPPlanResponse :datatype-) "dmp/GetDMPPlanResponse")
(setf (get dmp::GetDMPPlanResponse :definition-)
      "
float64[] x_0


float64[] x_dot_0



float64 t_0


float64[] goal





float64[] goal_thresh


float64 seg_length


float64 tau


float64 dt


int32 integrate_iter

---


DMPTraj plan


uint8 at_goal






================================================================================
MSG: dmp/DMPTraj
# points and times should be the same length
DMPPoint[] points

# Times of observations, in seconds, starting at zero
float64[] times



================================================================================
MSG: dmp/DMPPoint
# Positions and velocities of DOFs
#Velocity is only used for movement plans, not for giving demonstrations.
float64[] positions
float64[] velocities


")



(provide :dmp/GetDMPPlan "5cd79fd80676a4f8f062c5472a3190b1")


