;; Auto-generated. Do not edit!


(when (boundp 'robotino_msgs::ResetOdometry)
  (if (not (find-package "ROBOTINO_MSGS"))
    (make-package "ROBOTINO_MSGS"))
  (shadow 'ResetOdometry (find-package "ROBOTINO_MSGS")))
(unless (find-package "ROBOTINO_MSGS::RESETODOMETRY")
  (make-package "ROBOTINO_MSGS::RESETODOMETRY"))
(unless (find-package "ROBOTINO_MSGS::RESETODOMETRYREQUEST")
  (make-package "ROBOTINO_MSGS::RESETODOMETRYREQUEST"))
(unless (find-package "ROBOTINO_MSGS::RESETODOMETRYRESPONSE")
  (make-package "ROBOTINO_MSGS::RESETODOMETRYRESPONSE"))

(in-package "ROS")





(defclass robotino_msgs::ResetOdometryRequest
  :super ros::object
  :slots (_x _y _phi ))

(defmethod robotino_msgs::ResetOdometryRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:phi __phi) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _phi (float __phi))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:phi
   (&optional __phi)
   (if __phi (setq _phi __phi)) _phi)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _phi
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _phi
       (sys::poke _phi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _phi
     (setq _phi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass robotino_msgs::ResetOdometryResponse
  :super ros::object
  :slots ())

(defmethod robotino_msgs::ResetOdometryResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass robotino_msgs::ResetOdometry
  :super ros::object
  :slots ())

(setf (get robotino_msgs::ResetOdometry :md5sum-) "400210218ff66e5bce9e5be16653c0c8")
(setf (get robotino_msgs::ResetOdometry :datatype-) "robotino_msgs/ResetOdometry")
(setf (get robotino_msgs::ResetOdometry :request) robotino_msgs::ResetOdometryRequest)
(setf (get robotino_msgs::ResetOdometry :response) robotino_msgs::ResetOdometryResponse)

(defmethod robotino_msgs::ResetOdometryRequest
  (:response () (instance robotino_msgs::ResetOdometryResponse :init)))

(setf (get robotino_msgs::ResetOdometryRequest :md5sum-) "400210218ff66e5bce9e5be16653c0c8")
(setf (get robotino_msgs::ResetOdometryRequest :datatype-) "robotino_msgs/ResetOdometryRequest")
(setf (get robotino_msgs::ResetOdometryRequest :definition-)
      "float32 x # in meters
float32 y # in meters
float32 phi # in rad
---
")

(setf (get robotino_msgs::ResetOdometryResponse :md5sum-) "400210218ff66e5bce9e5be16653c0c8")
(setf (get robotino_msgs::ResetOdometryResponse :datatype-) "robotino_msgs/ResetOdometryResponse")
(setf (get robotino_msgs::ResetOdometryResponse :definition-)
      "float32 x # in meters
float32 y # in meters
float32 phi # in rad
---
")



(provide :robotino_msgs/ResetOdometry "400210218ff66e5bce9e5be16653c0c8")


