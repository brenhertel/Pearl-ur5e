
(cl:in-package :asdf)

(defsystem "dmp-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :dmp-msg
)
  :components ((:file "_package")
    (:file "GetDMPPlan" :depends-on ("_package_GetDMPPlan"))
    (:file "_package_GetDMPPlan" :depends-on ("_package"))
    (:file "LearnDMPFromDemo" :depends-on ("_package_LearnDMPFromDemo"))
    (:file "_package_LearnDMPFromDemo" :depends-on ("_package"))
    (:file "SetActiveDMP" :depends-on ("_package_SetActiveDMP"))
    (:file "_package_SetActiveDMP" :depends-on ("_package"))
  ))