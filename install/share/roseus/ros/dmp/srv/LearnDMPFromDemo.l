;; Auto-generated. Do not edit!


(when (boundp 'dmp::LearnDMPFromDemo)
  (if (not (find-package "DMP"))
    (make-package "DMP"))
  (shadow 'LearnDMPFromDemo (find-package "DMP")))
(unless (find-package "DMP::LEARNDMPFROMDEMO")
  (make-package "DMP::LEARNDMPFROMDEMO"))
(unless (find-package "DMP::LEARNDMPFROMDEMOREQUEST")
  (make-package "DMP::LEARNDMPFROMDEMOREQUEST"))
(unless (find-package "DMP::LEARNDMPFROMDEMORESPONSE")
  (make-package "DMP::LEARNDMPFROMDEMORESPONSE"))

(in-package "ROS")





(defclass dmp::LearnDMPFromDemoRequest
  :super ros::object
  :slots (_demo _k_gains _d_gains _num_bases ))

(defmethod dmp::LearnDMPFromDemoRequest
  (:init
   (&key
    ((:demo __demo) (instance dmp::DMPTraj :init))
    ((:k_gains __k_gains) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:d_gains __d_gains) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:num_bases __num_bases) 0)
    )
   (send-super :init)
   (setq _demo __demo)
   (setq _k_gains __k_gains)
   (setq _d_gains __d_gains)
   (setq _num_bases (round __num_bases))
   self)
  (:demo
   (&rest __demo)
   (if (keywordp (car __demo))
       (send* _demo __demo)
     (progn
       (if __demo (setq _demo (car __demo)))
       _demo)))
  (:k_gains
   (&optional __k_gains)
   (if __k_gains (setq _k_gains __k_gains)) _k_gains)
  (:d_gains
   (&optional __d_gains)
   (if __d_gains (setq _d_gains __d_gains)) _d_gains)
  (:num_bases
   (&optional __num_bases)
   (if __num_bases (setq _num_bases __num_bases)) _num_bases)
  (:serialization-length
   ()
   (+
    ;; dmp/DMPTraj _demo
    (send _demo :serialization-length)
    ;; float64[] _k_gains
    (* 8    (length _k_gains)) 4
    ;; float64[] _d_gains
    (* 8    (length _d_gains)) 4
    ;; int32 _num_bases
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; dmp/DMPTraj _demo
       (send _demo :serialize s)
     ;; float64[] _k_gains
     (write-long (length _k_gains) s)
     (dotimes (i (length _k_gains))
       (sys::poke (elt _k_gains i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _d_gains
     (write-long (length _d_gains) s)
     (dotimes (i (length _d_gains))
       (sys::poke (elt _d_gains i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _num_bases
       (write-long _num_bases s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; dmp/DMPTraj _demo
     (send _demo :deserialize buf ptr-) (incf ptr- (send _demo :serialization-length))
   ;; float64[] _k_gains
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _k_gains (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _k_gains i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _d_gains
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _d_gains (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _d_gains i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int32 _num_bases
     (setq _num_bases (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dmp::LearnDMPFromDemoResponse
  :super ros::object
  :slots (_dmp_list _tau ))

(defmethod dmp::LearnDMPFromDemoResponse
  (:init
   (&key
    ((:dmp_list __dmp_list) (let (r) (dotimes (i 0) (push (instance dmp::DMPData :init) r)) r))
    ((:tau __tau) 0.0)
    )
   (send-super :init)
   (setq _dmp_list __dmp_list)
   (setq _tau (float __tau))
   self)
  (:dmp_list
   (&rest __dmp_list)
   (if (keywordp (car __dmp_list))
       (send* _dmp_list __dmp_list)
     (progn
       (if __dmp_list (setq _dmp_list (car __dmp_list)))
       _dmp_list)))
  (:tau
   (&optional __tau)
   (if __tau (setq _tau __tau)) _tau)
  (:serialization-length
   ()
   (+
    ;; dmp/DMPData[] _dmp_list
    (apply #'+ (send-all _dmp_list :serialization-length)) 4
    ;; float64 _tau
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; dmp/DMPData[] _dmp_list
     (write-long (length _dmp_list) s)
     (dolist (elem _dmp_list)
       (send elem :serialize s)
       )
     ;; float64 _tau
       (sys::poke _tau (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; dmp/DMPData[] _dmp_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dmp_list (let (r) (dotimes (i n) (push (instance dmp::DMPData :init) r)) r))
     (dolist (elem- _dmp_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64 _tau
     (setq _tau (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass dmp::LearnDMPFromDemo
  :super ros::object
  :slots ())

(setf (get dmp::LearnDMPFromDemo :md5sum-) "3ba13cfa47585560a2fd9cc202efdbff")
(setf (get dmp::LearnDMPFromDemo :datatype-) "dmp/LearnDMPFromDemo")
(setf (get dmp::LearnDMPFromDemo :request) dmp::LearnDMPFromDemoRequest)
(setf (get dmp::LearnDMPFromDemo :response) dmp::LearnDMPFromDemoResponse)

(defmethod dmp::LearnDMPFromDemoRequest
  (:response () (instance dmp::LearnDMPFromDemoResponse :init)))

(setf (get dmp::LearnDMPFromDemoRequest :md5sum-) "3ba13cfa47585560a2fd9cc202efdbff")
(setf (get dmp::LearnDMPFromDemoRequest :datatype-) "dmp/LearnDMPFromDemoRequest")
(setf (get dmp::LearnDMPFromDemoRequest :definition-)
      "
DMPTraj demo


float64[] k_gains
float64[] d_gains


int32 num_bases


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


---


DMPData[] dmp_list


float64 tau


================================================================================
MSG: dmp/DMPData
float64 k_gain
float64 d_gain
float64[] weights
float64[] f_domain
float64[] f_targets

")

(setf (get dmp::LearnDMPFromDemoResponse :md5sum-) "3ba13cfa47585560a2fd9cc202efdbff")
(setf (get dmp::LearnDMPFromDemoResponse :datatype-) "dmp/LearnDMPFromDemoResponse")
(setf (get dmp::LearnDMPFromDemoResponse :definition-)
      "
DMPTraj demo


float64[] k_gains
float64[] d_gains


int32 num_bases


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


---


DMPData[] dmp_list


float64 tau


================================================================================
MSG: dmp/DMPData
float64 k_gain
float64 d_gain
float64[] weights
float64[] f_domain
float64[] f_targets

")



(provide :dmp/LearnDMPFromDemo "3ba13cfa47585560a2fd9cc202efdbff")


