;; Auto-generated. Do not edit!


(when (boundp 'dmp::DMPTraj)
  (if (not (find-package "DMP"))
    (make-package "DMP"))
  (shadow 'DMPTraj (find-package "DMP")))
(unless (find-package "DMP::DMPTRAJ")
  (make-package "DMP::DMPTRAJ"))

(in-package "ROS")
;;//! \htmlinclude DMPTraj.msg.html


(defclass dmp::DMPTraj
  :super ros::object
  :slots (_points _times ))

(defmethod dmp::DMPTraj
  (:init
   (&key
    ((:points __points) (let (r) (dotimes (i 0) (push (instance dmp::DMPPoint :init) r)) r))
    ((:times __times) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _points __points)
   (setq _times __times)
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:times
   (&optional __times)
   (if __times (setq _times __times)) _times)
  (:serialization-length
   ()
   (+
    ;; dmp/DMPPoint[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ;; float64[] _times
    (* 8    (length _times)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; dmp/DMPPoint[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;; float64[] _times
     (write-long (length _times) s)
     (dotimes (i (length _times))
       (sys::poke (elt _times i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; dmp/DMPPoint[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance dmp::DMPPoint :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _times
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _times (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _times i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get dmp::DMPTraj :md5sum-) "1d088d86ab60cf6a2671bc3c0e99932b")
(setf (get dmp::DMPTraj :datatype-) "dmp/DMPTraj")
(setf (get dmp::DMPTraj :definition-)
      "# points and times should be the same length
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



(provide :dmp/DMPTraj "1d088d86ab60cf6a2671bc3c0e99932b")


