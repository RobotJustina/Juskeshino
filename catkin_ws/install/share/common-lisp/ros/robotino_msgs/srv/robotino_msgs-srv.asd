
(cl:in-package :asdf)

(defsystem "robotino_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ResetOdometry" :depends-on ("_package_ResetOdometry"))
    (:file "_package_ResetOdometry" :depends-on ("_package"))
    (:file "SetEncoderPosition" :depends-on ("_package_SetEncoderPosition"))
    (:file "_package_SetEncoderPosition" :depends-on ("_package"))
    (:file "SetGripperState" :depends-on ("_package_SetGripperState"))
    (:file "_package_SetGripperState" :depends-on ("_package"))
  ))