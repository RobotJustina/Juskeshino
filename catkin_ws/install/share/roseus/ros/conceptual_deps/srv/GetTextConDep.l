;; Auto-generated. Do not edit!


(when (boundp 'conceptual_deps::GetTextConDep)
  (if (not (find-package "CONCEPTUAL_DEPS"))
    (make-package "CONCEPTUAL_DEPS"))
  (shadow 'GetTextConDep (find-package "CONCEPTUAL_DEPS")))
(unless (find-package "CONCEPTUAL_DEPS::GETTEXTCONDEP")
  (make-package "CONCEPTUAL_DEPS::GETTEXTCONDEP"))
(unless (find-package "CONCEPTUAL_DEPS::GETTEXTCONDEPREQUEST")
  (make-package "CONCEPTUAL_DEPS::GETTEXTCONDEPREQUEST"))
(unless (find-package "CONCEPTUAL_DEPS::GETTEXTCONDEPRESPONSE")
  (make-package "CONCEPTUAL_DEPS::GETTEXTCONDEPRESPONSE"))

(in-package "ROS")





(defclass conceptual_deps::GetTextConDepRequest
  :super ros::object
  :slots (_text ))

(defmethod conceptual_deps::GetTextConDepRequest
  (:init
   (&key
    ((:text __text) "")
    )
   (send-super :init)
   (setq _text (string __text))
   self)
  (:text
   (&optional __text)
   (if __text (setq _text __text)) _text)
  (:serialization-length
   ()
   (+
    ;; string _text
    4 (length _text)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _text
       (write-long (length _text) s) (princ _text s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass conceptual_deps::GetTextConDepResponse
  :super ros::object
  :slots (_cds ))

(defmethod conceptual_deps::GetTextConDepResponse
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

(defclass conceptual_deps::GetTextConDep
  :super ros::object
  :slots ())

(setf (get conceptual_deps::GetTextConDep :md5sum-) "b7e9f26875b56c920fd22b0d43671c0b")
(setf (get conceptual_deps::GetTextConDep :datatype-) "conceptual_deps/GetTextConDep")
(setf (get conceptual_deps::GetTextConDep :request) conceptual_deps::GetTextConDepRequest)
(setf (get conceptual_deps::GetTextConDep :response) conceptual_deps::GetTextConDepResponse)

(defmethod conceptual_deps::GetTextConDepRequest
  (:response () (instance conceptual_deps::GetTextConDepResponse :init)))

(setf (get conceptual_deps::GetTextConDepRequest :md5sum-) "b7e9f26875b56c920fd22b0d43671c0b")
(setf (get conceptual_deps::GetTextConDepRequest :datatype-) "conceptual_deps/GetTextConDepRequest")
(setf (get conceptual_deps::GetTextConDepRequest :definition-)
      "string text
---
StringArray  cds


================================================================================
MSG: conceptual_deps/StringArray
string[]  data
")

(setf (get conceptual_deps::GetTextConDepResponse :md5sum-) "b7e9f26875b56c920fd22b0d43671c0b")
(setf (get conceptual_deps::GetTextConDepResponse :datatype-) "conceptual_deps/GetTextConDepResponse")
(setf (get conceptual_deps::GetTextConDepResponse :definition-)
      "string text
---
StringArray  cds


================================================================================
MSG: conceptual_deps/StringArray
string[]  data
")



(provide :conceptual_deps/GetTextConDep "b7e9f26875b56c920fd22b0d43671c0b")


