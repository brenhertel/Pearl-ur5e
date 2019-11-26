;; Auto-generated. Do not edit!


(when (boundp 'dmp::DMPData)
  (if (not (find-package "DMP"))
    (make-package "DMP"))
  (shadow 'DMPData (find-package "DMP")))
(unless (find-package "DMP::DMPDATA")
  (make-package "DMP::DMPDATA"))

(in-package "ROS")
;;//! \htmlinclude DMPData.msg.html


(defclass dmp::DMPData
  :super ros::object
  :slots (_k_gain _d_gain _weights _f_domain _f_targets ))

(defmethod dmp::DMPData
  (:init
   (&key
    ((:k_gain __k_gain) 0.0)
    ((:d_gain __d_gain) 0.0)
    ((:weights __weights) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_domain __f_domain) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:f_targets __f_targets) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _k_gain (float __k_gain))
   (setq _d_gain (float __d_gain))
   (setq _weights __weights)
   (setq _f_domain __f_domain)
   (setq _f_targets __f_targets)
   self)
  (:k_gain
   (&optional __k_gain)
   (if __k_gain (setq _k_gain __k_gain)) _k_gain)
  (:d_gain
   (&optional __d_gain)
   (if __d_gain (setq _d_gain __d_gain)) _d_gain)
  (:weights
   (&optional __weights)
   (if __weights (setq _weights __weights)) _weights)
  (:f_domain
   (&optional __f_domain)
   (if __f_domain (setq _f_domain __f_domain)) _f_domain)
  (:f_targets
   (&optional __f_targets)
   (if __f_targets (setq _f_targets __f_targets)) _f_targets)
  (:serialization-length
   ()
   (+
    ;; float64 _k_gain
    8
    ;; float64 _d_gain
    8
    ;; float64[] _weights
    (* 8    (length _weights)) 4
    ;; float64[] _f_domain
    (* 8    (length _f_domain)) 4
    ;; float64[] _f_targets
    (* 8    (length _f_targets)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _k_gain
       (sys::poke _k_gain (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d_gain
       (sys::poke _d_gain (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _weights
     (write-long (length _weights) s)
     (dotimes (i (length _weights))
       (sys::poke (elt _weights i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _f_domain
     (write-long (length _f_domain) s)
     (dotimes (i (length _f_domain))
       (sys::poke (elt _f_domain i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _f_targets
     (write-long (length _f_targets) s)
     (dotimes (i (length _f_targets))
       (sys::poke (elt _f_targets i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _k_gain
     (setq _k_gain (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d_gain
     (setq _d_gain (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _weights
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _weights (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _weights i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _f_domain
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_domain (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_domain i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _f_targets
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _f_targets (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _f_targets i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get dmp::DMPData :md5sum-) "dd2a2dd30705ac6b894c0c83b8081221")
(setf (get dmp::DMPData :datatype-) "dmp/DMPData")
(setf (get dmp::DMPData :definition-)
      "float64 k_gain
float64 d_gain
float64[] weights
float64[] f_domain
float64[] f_targets


")



(provide :dmp/DMPData "dd2a2dd30705ac6b894c0c83b8081221")


