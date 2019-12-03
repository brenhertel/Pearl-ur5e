
(cl:in-package :asdf)

(defsystem "brendan_ur5e-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "shutdown_request" :depends-on ("_package_shutdown_request"))
    (:file "_package_shutdown_request" :depends-on ("_package"))
    (:file "time_shutdown_request" :depends-on ("_package_time_shutdown_request"))
    (:file "_package_time_shutdown_request" :depends-on ("_package"))
  ))