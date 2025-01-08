;; Auto-generated. Do not edit!


(when (boundp 'robotino_msgs::MotorReadings)
  (if (not (find-package "ROBOTINO_MSGS"))
    (make-package "ROBOTINO_MSGS"))
  (shadow 'MotorReadings (find-package "ROBOTINO_MSGS")))
(unless (find-package "ROBOTINO_MSGS::MOTORREADINGS")
  (make-package "ROBOTINO_MSGS::MOTORREADINGS"))

(in-package "ROS")
;;//! \htmlinclude MotorReadings.msg.html


(defclass robotino_msgs::MotorReadings
  :super ros::object
  :slots (_stamp _velocities _positions _currents ))

(defmethod robotino_msgs::MotorReadings
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:velocities __velocities) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:positions __positions) (make-array 0 :initial-element 0 :element-type :integer))
    ((:currents __currents) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _velocities __velocities)
   (setq _positions __positions)
   (setq _currents __currents)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:velocities
   (&optional __velocities)
   (if __velocities (setq _velocities __velocities)) _velocities)
  (:positions
   (&optional __positions)
   (if __positions (setq _positions __positions)) _positions)
  (:currents
   (&optional __currents)
   (if __currents (setq _currents __currents)) _currents)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; float32[] _velocities
    (* 4    (length _velocities)) 4
    ;; int32[] _positions
    (* 4    (length _positions)) 4
    ;; float32[] _currents
    (* 4    (length _currents)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; float32[] _velocities
     (write-long (length _velocities) s)
     (dotimes (i (length _velocities))
       (sys::poke (elt _velocities i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int32[] _positions
     (write-long (length _positions) s)
     (dotimes (i (length _positions))
       (write-long (elt _positions i) s)
       )
     ;; float32[] _currents
     (write-long (length _currents) s)
     (dotimes (i (length _currents))
       (sys::poke (elt _currents i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _velocities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocities i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int32[] _positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _positions i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _currents
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _currents (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _currents i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get robotino_msgs::MotorReadings :md5sum-) "3974e9bd8305667fc0637697b49a8e1f")
(setf (get robotino_msgs::MotorReadings :datatype-) "robotino_msgs/MotorReadings")
(setf (get robotino_msgs::MotorReadings :definition-)
      "time stamp
float32[] velocities 	# in rpm
int32[] positions
float32[] currents		# in A
")



(provide :robotino_msgs/MotorReadings "3974e9bd8305667fc0637697b49a8e1f")


