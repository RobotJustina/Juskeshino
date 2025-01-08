;; Auto-generated. Do not edit!


(when (boundp 'robotino_msgs::PowerReadings)
  (if (not (find-package "ROBOTINO_MSGS"))
    (make-package "ROBOTINO_MSGS"))
  (shadow 'PowerReadings (find-package "ROBOTINO_MSGS")))
(unless (find-package "ROBOTINO_MSGS::POWERREADINGS")
  (make-package "ROBOTINO_MSGS::POWERREADINGS"))

(in-package "ROS")
;;//! \htmlinclude PowerReadings.msg.html


(defclass robotino_msgs::PowerReadings
  :super ros::object
  :slots (_stamp _current _voltage ))

(defmethod robotino_msgs::PowerReadings
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:current __current) 0.0)
    ((:voltage __voltage) 0.0)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _current (float __current))
   (setq _voltage (float __voltage))
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; float32 _current
    4
    ;; float32 _voltage
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; float32 _current
       (sys::poke _current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _current
     (setq _current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _voltage
     (setq _voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robotino_msgs::PowerReadings :md5sum-) "a00aebd29e1775a226f3fa330a4fec59")
(setf (get robotino_msgs::PowerReadings :datatype-) "robotino_msgs/PowerReadings")
(setf (get robotino_msgs::PowerReadings :definition-)
      "time stamp	# time-stamp
float32 current	# current in A
float32 voltage	# voltage in V

")



(provide :robotino_msgs/PowerReadings "a00aebd29e1775a226f3fa330a4fec59")


