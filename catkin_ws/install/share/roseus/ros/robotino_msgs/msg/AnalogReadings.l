;; Auto-generated. Do not edit!


(when (boundp 'robotino_msgs::AnalogReadings)
  (if (not (find-package "ROBOTINO_MSGS"))
    (make-package "ROBOTINO_MSGS"))
  (shadow 'AnalogReadings (find-package "ROBOTINO_MSGS")))
(unless (find-package "ROBOTINO_MSGS::ANALOGREADINGS")
  (make-package "ROBOTINO_MSGS::ANALOGREADINGS"))

(in-package "ROS")
;;//! \htmlinclude AnalogReadings.msg.html


(defclass robotino_msgs::AnalogReadings
  :super ros::object
  :slots (_stamp _values ))

(defmethod robotino_msgs::AnalogReadings
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:values __values) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _values __values)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:values
   (&optional __values)
   (if __values (setq _values __values)) _values)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; float32[] _values
    (* 4    (length _values)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; float32[] _values
     (write-long (length _values) s)
     (dotimes (i (length _values))
       (sys::poke (elt _values i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _values i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get robotino_msgs::AnalogReadings :md5sum-) "b18badc07ef45644f603212cc1654669")
(setf (get robotino_msgs::AnalogReadings :datatype-) "robotino_msgs/AnalogReadings")
(setf (get robotino_msgs::AnalogReadings :definition-)
      "time stamp
float32[] values
")



(provide :robotino_msgs/AnalogReadings "b18badc07ef45644f603212cc1654669")


