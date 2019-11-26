;; Auto-generated. Do not edit!


(when (boundp 'dmp::SetActiveDMP)
  (if (not (find-package "DMP"))
    (make-package "DMP"))
  (shadow 'SetActiveDMP (find-package "DMP")))
(unless (find-package "DMP::SETACTIVEDMP")
  (make-package "DMP::SETACTIVEDMP"))
(unless (find-package "DMP::SETACTIVEDMPREQUEST")
  (make-package "DMP::SETACTIVEDMPREQUEST"))
(unless (find-package "DMP::SETACTIVEDMPRESPONSE")
  (make-package "DMP::SETACTIVEDMPRESPONSE"))

(in-package "ROS")





(defclass dmp::SetActiveDMPRequest
  :super ros::object
  :slots (_dmp_list ))

(defmethod dmp::SetActiveDMPRequest
  (:init
   (&key
    ((:dmp_list __dmp_list) (let (r) (dotimes (i 0) (push (instance dmp::DMPData :init) r)) r))
    )
   (send-super :init)
   (setq _dmp_list __dmp_list)
   self)
  (:dmp_list
   (&rest __dmp_list)
   (if (keywordp (car __dmp_list))
       (send* _dmp_list __dmp_list)
     (progn
       (if __dmp_list (setq _dmp_list (car __dmp_list)))
       _dmp_list)))
  (:serialization-length
   ()
   (+
    ;; dmp/DMPData[] _dmp_list
    (apply #'+ (send-all _dmp_list :serialization-length)) 4
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
   ;;
   self)
  )

(defclass dmp::SetActiveDMPResponse
  :super ros::object
  :slots (_success ))

(defmethod dmp::SetActiveDMPResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dmp::SetActiveDMP
  :super ros::object
  :slots ())

(setf (get dmp::SetActiveDMP :md5sum-) "10d64adb0a08cbb7afbd425801f828e5")
(setf (get dmp::SetActiveDMP :datatype-) "dmp/SetActiveDMP")
(setf (get dmp::SetActiveDMP :request) dmp::SetActiveDMPRequest)
(setf (get dmp::SetActiveDMP :response) dmp::SetActiveDMPResponse)

(defmethod dmp::SetActiveDMPRequest
  (:response () (instance dmp::SetActiveDMPResponse :init)))

(setf (get dmp::SetActiveDMPRequest :md5sum-) "10d64adb0a08cbb7afbd425801f828e5")
(setf (get dmp::SetActiveDMPRequest :datatype-) "dmp/SetActiveDMPRequest")
(setf (get dmp::SetActiveDMPRequest :definition-)
      "
DMPData[] dmp_list


================================================================================
MSG: dmp/DMPData
float64 k_gain
float64 d_gain
float64[] weights
float64[] f_domain
float64[] f_targets

---


bool success





")

(setf (get dmp::SetActiveDMPResponse :md5sum-) "10d64adb0a08cbb7afbd425801f828e5")
(setf (get dmp::SetActiveDMPResponse :datatype-) "dmp/SetActiveDMPResponse")
(setf (get dmp::SetActiveDMPResponse :definition-)
      "
DMPData[] dmp_list


================================================================================
MSG: dmp/DMPData
float64 k_gain
float64 d_gain
float64[] weights
float64[] f_domain
float64[] f_targets

---


bool success





")



(provide :dmp/SetActiveDMP "10d64adb0a08cbb7afbd425801f828e5")


