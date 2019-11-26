;; Auto-generated. Do not edit!


(when (boundp 'dmp::DMPPoint)
  (if (not (find-package "DMP"))
    (make-package "DMP"))
  (shadow 'DMPPoint (find-package "DMP")))
(unless (find-package "DMP::DMPPOINT")
  (make-package "DMP::DMPPOINT"))

(in-package "ROS")
;;//! \htmlinclude DMPPoint.msg.html


(defclass dmp::DMPPoint
  :super ros::object
  :slots (_positions _velocities ))

(defmethod dmp::DMPPoint
  (:init
   (&key
    ((:positions __positions) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocities __velocities) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _positions __positions)
   (setq _velocities __velocities)
   self)
  (:positions
   (&optional __positions)
   (if __positions (setq _positions __positions)) _positions)
  (:velocities
   (&optional __velocities)
   (if __velocities (setq _velocities __velocities)) _velocities)
  (:serialization-length
   ()
   (+
    ;; float64[] _positions
    (* 8    (length _positions)) 4
    ;; float64[] _velocities
    (* 8    (length _velocities)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _positions
     (write-long (length _positions) s)
     (dotimes (i (length _positions))
       (sys::poke (elt _positions i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocities
     (write-long (length _velocities) s)
     (dotimes (i (length _velocities))
       (sys::poke (elt _velocities i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _positions i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocities i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get dmp::DMPPoint :md5sum-) "f32a98450510f8ae05fae28f45d6e99a")
(setf (get dmp::DMPPoint :datatype-) "dmp/DMPPoint")
(setf (get dmp::DMPPoint :definition-)
      "# Positions and velocities of DOFs
#Velocity is only used for movement plans, not for giving demonstrations.
float64[] positions
float64[] velocities



")



(provide :dmp/DMPPoint "f32a98450510f8ae05fae28f45d6e99a")


