
(cl:in-package :asdf)

(defsystem "dmp-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DMPData" :depends-on ("_package_DMPData"))
    (:file "_package_DMPData" :depends-on ("_package"))
    (:file "DMPPoint" :depends-on ("_package_DMPPoint"))
    (:file "_package_DMPPoint" :depends-on ("_package"))
    (:file "DMPTraj" :depends-on ("_package_DMPTraj"))
    (:file "_package_DMPTraj" :depends-on ("_package"))
  ))