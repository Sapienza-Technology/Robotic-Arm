
(cl:in-package :asdf)

(defsystem "tag_utils-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetTag" :depends-on ("_package_GetTag"))
    (:file "_package_GetTag" :depends-on ("_package"))
  ))