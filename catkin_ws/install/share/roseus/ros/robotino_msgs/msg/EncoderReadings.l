;; Auto-generated. Do not edit!


(when (boundp 'robotino_msgs::EncoderReadings)
  (if (not (find-package "ROBOTINO_MSGS"))
    (make-package "ROBOTINO_MSGS"))
  (shadow 'EncoderReadings (find-package "ROBOTINO_MSGS")))
(unless (find-package "ROBOTINO_MSGS::ENCODERREADINGS")
  (make-package "ROBOTINO_MSGS::ENCODERREADINGS"))

(in-package "ROS")
;;//! \htmlinclude EncoderReadings.msg.html


(defclass robotino_msgs::EncoderReadings
  :super ros::object
  :slots (_stamp _velocity _position _current ))

(defmethod robotino_msgs::EncoderReadings
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:velocity __velocity) 0)
    ((:position __position) 0)
    ((:current __current) 0)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _velocity (round __velocity))
   (setq _position (round __position))
   (setq _current (round __current))
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; uint32 _velocity
    4
    ;; uint32 _position
    4
    ;; uint32 _current
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; uint32 _velocity
       (write-long _velocity s)
     ;; uint32 _position
       (write-long _position s)
     ;; uint32 _current
       (write-long _current s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _velocity
     (setq _velocity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _current
     (setq _current (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robotino_msgs::EncoderReadings :md5sum-) "0b4033dda61bb04d3e2ea6c671f26183")
(setf (get robotino_msgs::EncoderReadings :datatype-) "robotino_msgs/EncoderReadings")
(setf (get robotino_msgs::EncoderReadings :definition-)
      "time stamp
uint32 velocity		# actual velocity in ticks/s
uint32 position		# actual position in ticks
uint32 current		# in A
")



(provide :robotino_msgs/EncoderReadings "0b4033dda61bb04d3e2ea6c671f26183")


