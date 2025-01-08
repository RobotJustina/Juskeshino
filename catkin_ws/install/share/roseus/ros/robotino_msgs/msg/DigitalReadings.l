;; Auto-generated. Do not edit!


(when (boundp 'robotino_msgs::DigitalReadings)
  (if (not (find-package "ROBOTINO_MSGS"))
    (make-package "ROBOTINO_MSGS"))
  (shadow 'DigitalReadings (find-package "ROBOTINO_MSGS")))
(unless (find-package "ROBOTINO_MSGS::DIGITALREADINGS")
  (make-package "ROBOTINO_MSGS::DIGITALREADINGS"))

(in-package "ROS")
;;//! \htmlinclude DigitalReadings.msg.html


(defclass robotino_msgs::DigitalReadings
  :super ros::object
  :slots (_stamp _values ))

(defmethod robotino_msgs::DigitalReadings
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:values __values) (let (r) (dotimes (i 0) (push nil r)) r))
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
    ;; bool[] _values
    (* 1    (length _values)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; bool[] _values
     (write-long (length _values) s)
     (dotimes (i (length _values))
       (if (elt _values i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool[] _values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _values (make-list n))
     (dotimes (i n)
     (setf (elt _values i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;;
   self)
  )

(setf (get robotino_msgs::DigitalReadings :md5sum-) "21240637a82d18c261b7e2f567659e7e")
(setf (get robotino_msgs::DigitalReadings :datatype-) "robotino_msgs/DigitalReadings")
(setf (get robotino_msgs::DigitalReadings :definition-)
      "time stamp
bool[] values
")



(provide :robotino_msgs/DigitalReadings "21240637a82d18c261b7e2f567659e7e")


