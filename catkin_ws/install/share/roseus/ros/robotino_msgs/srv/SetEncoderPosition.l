;; Auto-generated. Do not edit!


(when (boundp 'robotino_msgs::SetEncoderPosition)
  (if (not (find-package "ROBOTINO_MSGS"))
    (make-package "ROBOTINO_MSGS"))
  (shadow 'SetEncoderPosition (find-package "ROBOTINO_MSGS")))
(unless (find-package "ROBOTINO_MSGS::SETENCODERPOSITION")
  (make-package "ROBOTINO_MSGS::SETENCODERPOSITION"))
(unless (find-package "ROBOTINO_MSGS::SETENCODERPOSITIONREQUEST")
  (make-package "ROBOTINO_MSGS::SETENCODERPOSITIONREQUEST"))
(unless (find-package "ROBOTINO_MSGS::SETENCODERPOSITIONRESPONSE")
  (make-package "ROBOTINO_MSGS::SETENCODERPOSITIONRESPONSE"))

(in-package "ROS")





(defclass robotino_msgs::SetEncoderPositionRequest
  :super ros::object
  :slots (_position _velocity ))

(defmethod robotino_msgs::SetEncoderPositionRequest
  (:init
   (&key
    ((:position __position) 0)
    ((:velocity __velocity) 0)
    )
   (send-super :init)
   (setq _position (round __position))
   (setq _velocity (round __velocity))
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:serialization-length
   ()
   (+
    ;; uint32 _position
    4
    ;; uint32 _velocity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _position
       (write-long _position s)
     ;; uint32 _velocity
       (write-long _velocity s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _velocity
     (setq _velocity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass robotino_msgs::SetEncoderPositionResponse
  :super ros::object
  :slots ())

(defmethod robotino_msgs::SetEncoderPositionResponse
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

(defclass robotino_msgs::SetEncoderPosition
  :super ros::object
  :slots ())

(setf (get robotino_msgs::SetEncoderPosition :md5sum-) "e17f4b08b3e2cdbbf2b0d78e4f62e5d6")
(setf (get robotino_msgs::SetEncoderPosition :datatype-) "robotino_msgs/SetEncoderPosition")
(setf (get robotino_msgs::SetEncoderPosition :request) robotino_msgs::SetEncoderPositionRequest)
(setf (get robotino_msgs::SetEncoderPosition :response) robotino_msgs::SetEncoderPositionResponse)

(defmethod robotino_msgs::SetEncoderPositionRequest
  (:response () (instance robotino_msgs::SetEncoderPositionResponse :init)))

(setf (get robotino_msgs::SetEncoderPositionRequest :md5sum-) "e17f4b08b3e2cdbbf2b0d78e4f62e5d6")
(setf (get robotino_msgs::SetEncoderPositionRequest :datatype-) "robotino_msgs/SetEncoderPositionRequest")
(setf (get robotino_msgs::SetEncoderPositionRequest :definition-)
      "uint32 position		# in encoder ticks
uint32 velocity		# in ticks/s
---
")

(setf (get robotino_msgs::SetEncoderPositionResponse :md5sum-) "e17f4b08b3e2cdbbf2b0d78e4f62e5d6")
(setf (get robotino_msgs::SetEncoderPositionResponse :datatype-) "robotino_msgs/SetEncoderPositionResponse")
(setf (get robotino_msgs::SetEncoderPositionResponse :definition-)
      "uint32 position		# in encoder ticks
uint32 velocity		# in ticks/s
---
")



(provide :robotino_msgs/SetEncoderPosition "e17f4b08b3e2cdbbf2b0d78e4f62e5d6")


