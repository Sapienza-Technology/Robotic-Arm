
(cl:in-package :asdf)

(defsystem "braccio_urdf_description-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Float6Array" :depends-on ("_package_Float6Array"))
    (:file "_package_Float6Array" :depends-on ("_package"))
  ))