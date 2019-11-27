;; Auto-generated. Do not edit!


(when (boundp 'brendan_ur5e::shutdown_request)
  (if (not (find-package "BRENDAN_UR5E"))
    (make-package "BRENDAN_UR5E"))
  (shadow 'shutdown_request (find-package "BRENDAN_UR5E")))
(unless (find-package "BRENDAN_UR5E::SHUTDOWN_REQUEST")
  (make-package "BRENDAN_UR5E::SHUTDOWN_REQUEST"))
(unless (find-package "BRENDAN_UR5E::SHUTDOWN_REQUESTREQUEST")
  (make-package "BRENDAN_UR5E::SHUTDOWN_REQUESTREQUEST"))
(unless (find-package "BRENDAN_UR5E::SHUTDOWN_REQUESTRESPONSE")
  (make-package "BRENDAN_UR5E::SHUTDOWN_REQUESTRESPONSE"))

(in-package "ROS")





(defclass brendan_ur5e::shutdown_requestRequest
  :super ros::object
  :slots (_a ))

(defmethod brendan_ur5e::shutdown_requestRequest
  (:init
   (&key
    ((:a __a) 0)
    )
   (send-super :init)
   (setq _a (round __a))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:serialization-length
   ()
   (+
    ;; int64 _a
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _a
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _a (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _a) (= (length (_a . bv)) 2)) ;; bignum
              (write-long (ash (elt (_a . bv) 0) 0) s)
              (write-long (ash (elt (_a . bv) 1) -1) s))
             ((and (class _a) (= (length (_a . bv)) 1)) ;; big1
              (write-long (elt (_a . bv) 0) s)
              (write-long (if (>= _a 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _a s)(write-long (if (>= _a 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _a
#+(or :alpha :irix6 :x86_64)
      (setf _a (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _a (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass brendan_ur5e::shutdown_requestResponse
  :super ros::object
  :slots ())

(defmethod brendan_ur5e::shutdown_requestResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass brendan_ur5e::shutdown_request
  :super ros::object
  :slots ())

(setf (get brendan_ur5e::shutdown_request :md5sum-) "019706110004b728d56d8baaa8e32797")
(setf (get brendan_ur5e::shutdown_request :datatype-) "brendan_ur5e/shutdown_request")
(setf (get brendan_ur5e::shutdown_request :request) brendan_ur5e::shutdown_requestRequest)
(setf (get brendan_ur5e::shutdown_request :response) brendan_ur5e::shutdown_requestResponse)

(defmethod brendan_ur5e::shutdown_requestRequest
  (:response () (instance brendan_ur5e::shutdown_requestResponse :init)))

(setf (get brendan_ur5e::shutdown_requestRequest :md5sum-) "019706110004b728d56d8baaa8e32797")
(setf (get brendan_ur5e::shutdown_requestRequest :datatype-) "brendan_ur5e/shutdown_requestRequest")
(setf (get brendan_ur5e::shutdown_requestRequest :definition-)
      "int64 a
---

")

(setf (get brendan_ur5e::shutdown_requestResponse :md5sum-) "019706110004b728d56d8baaa8e32797")
(setf (get brendan_ur5e::shutdown_requestResponse :datatype-) "brendan_ur5e/shutdown_requestResponse")
(setf (get brendan_ur5e::shutdown_requestResponse :definition-)
      "int64 a
---

")



(provide :brendan_ur5e/shutdown_request "019706110004b728d56d8baaa8e32797")


