
(cl:in-package :asdf)

(defsystem "ibvs_formation_bearing-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "bearing" :depends-on ("_package_bearing"))
    (:file "_package_bearing" :depends-on ("_package"))
  ))