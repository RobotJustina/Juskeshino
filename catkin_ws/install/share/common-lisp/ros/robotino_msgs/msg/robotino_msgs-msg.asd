
(cl:in-package :asdf)

(defsystem "robotino_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AnalogReadings" :depends-on ("_package_AnalogReadings"))
    (:file "_package_AnalogReadings" :depends-on ("_package"))
    (:file "DigitalReadings" :depends-on ("_package_DigitalReadings"))
    (:file "_package_DigitalReadings" :depends-on ("_package"))
    (:file "EncoderReadings" :depends-on ("_package_EncoderReadings"))
    (:file "_package_EncoderReadings" :depends-on ("_package"))
    (:file "MotorReadings" :depends-on ("_package_MotorReadings"))
    (:file "_package_MotorReadings" :depends-on ("_package"))
    (:file "PowerReadings" :depends-on ("_package_PowerReadings"))
    (:file "_package_PowerReadings" :depends-on ("_package"))
  ))