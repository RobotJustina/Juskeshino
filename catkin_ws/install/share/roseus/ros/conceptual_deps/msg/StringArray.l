;; Auto-generated. Do not edit!


(when (boundp 'conceptual_deps::StringArray)
  (if (not (find-package "CONCEPTUAL_DEPS"))
    (make-package "CONCEPTUAL_DEPS"))
  (shadow 'StringArray (find-package "CONCEPTUAL_DEPS")))
(unless (find-package "CONCEPTUAL_DEPS::STRINGARRAY")
  (make-package "CONCEPTUAL_DEPS::STRINGARRAY"))

(in-package "ROS")
;;//! \htmlinclude StringArray.msg.html


(defclass conceptual_deps::StringArray
  :super ros::object
  :slots (_data ))

(defmethod conceptual_deps::StringArray
  (:init
   (&key
    ((:data __data) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string[] _data
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _data i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get conceptual_deps::StringArray :md5sum-) "cce5a364f3a3be12c9722c6dcad2fa94")
(setf (get conceptual_deps::StringArray :datatype-) "conceptual_deps/StringArray")
(setf (get conceptual_deps::StringArray :definition-)
      "string[]  data

")



(provide :conceptual_deps/StringArray "cce5a364f3a3be12c9722c6dcad2fa94")


