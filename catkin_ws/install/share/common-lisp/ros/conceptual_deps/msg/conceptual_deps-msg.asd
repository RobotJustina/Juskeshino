
(cl:in-package :asdf)

(defsystem "conceptual_deps-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "StringArray" :depends-on ("_package_StringArray"))
    (:file "_package_StringArray" :depends-on ("_package"))
  ))