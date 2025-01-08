;; Auto-generated. Do not edit!


(when (boundp 'conceptual_deps::GetConDep)
  (if (not (find-package "CONCEPTUAL_DEPS"))
    (make-package "CONCEPTUAL_DEPS"))
  (shadow 'GetConDep (find-package "CONCEPTUAL_DEPS")))
(unless (find-package "CONCEPTUAL_DEPS::GETCONDEP")
  (make-package "CONCEPTUAL_DEPS::GETCONDEP"))
(unless (find-package "CONCEPTUAL_DEPS::GETCONDEPREQUEST")
  (make-package "CONCEPTUAL_DEPS::GETCONDEPREQUEST"))
(unless (find-package "CONCEPTUAL_DEPS::GETCONDEPRESPONSE")
  (make-package "CONCEPTUAL_DEPS::GETCONDEPRESPONSE"))

(in-package "ROS")





(defclass conceptual_deps::GetConDepRequest
  :super ros::object
  :slots ())

(defmethod conceptual_deps::GetConDepRequest
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

(defclass conceptual_deps::GetConDepResponse
  :super ros::object
  :slots (_cds ))

(defmethod conceptual_deps::GetConDepResponse
  (:init
   (&key
    ((:cds __cds) (instance conceptual_deps::StringArray :init))
    )
   (send-super :init)
   (setq _cds __cds)
   self)
  (:cds
   (&rest __cds)
   (if (keywordp (car __cds))
       (send* _cds __cds)
     (progn
       (if __cds (setq _cds (car __cds)))
       _cds)))
  (:serialization-length
   ()
   (+
    ;; conceptual_deps/StringArray _cds
    (send _cds :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; conceptual_deps/StringArray _cds
       (send _cds :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; conceptual_deps/StringArray _cds
     (send _cds :deserialize buf ptr-) (incf ptr- (send _cds :serialization-length))
   ;;
   self)
  )

(defclass conceptual_deps::GetConDep
  :super ros::object
  :slots ())

(setf (get conceptual_deps::GetConDep :md5sum-) "6530474bcb00416881ea95591006435a")
(setf (get conceptual_deps::GetConDep :datatype-) "conceptual_deps/GetConDep")
(setf (get conceptual_deps::GetConDep :request) conceptual_deps::GetConDepRequest)
(setf (get conceptual_deps::GetConDep :response) conceptual_deps::GetConDepResponse)

(defmethod conceptual_deps::GetConDepRequest
  (:response () (instance conceptual_deps::GetConDepResponse :init)))

(setf (get conceptual_deps::GetConDepRequest :md5sum-) "6530474bcb00416881ea95591006435a")
(setf (get conceptual_deps::GetConDepRequest :datatype-) "conceptual_deps/GetConDepRequest")
(setf (get conceptual_deps::GetConDepRequest :definition-)
      "---
StringArray  cds


================================================================================
MSG: conceptual_deps/StringArray
string[]  data
")

(setf (get conceptual_deps::GetConDepResponse :md5sum-) "6530474bcb00416881ea95591006435a")
(setf (get conceptual_deps::GetConDepResponse :datatype-) "conceptual_deps/GetConDepResponse")
(setf (get conceptual_deps::GetConDepResponse :definition-)
      "---
StringArray  cds


================================================================================
MSG: conceptual_deps/StringArray
string[]  data
")



(provide :conceptual_deps/GetConDep "6530474bcb00416881ea95591006435a")


