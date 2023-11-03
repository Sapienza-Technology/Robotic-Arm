
(cl:in-package :asdf)

(defsystem "arm_old_and_auxiliary_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Float6Array" :depends-on ("_package_Float6Array"))
    (:file "_package_Float6Array" :depends-on ("_package"))
  ))