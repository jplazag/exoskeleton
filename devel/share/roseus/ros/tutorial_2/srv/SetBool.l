;; Auto-generated. Do not edit!


(when (boundp 'tutorial_2::SetBool)
  (if (not (find-package "TUTORIAL_2"))
    (make-package "TUTORIAL_2"))
  (shadow 'SetBool (find-package "TUTORIAL_2")))
(unless (find-package "TUTORIAL_2::SETBOOL")
  (make-package "TUTORIAL_2::SETBOOL"))
(unless (find-package "TUTORIAL_2::SETBOOLREQUEST")
  (make-package "TUTORIAL_2::SETBOOLREQUEST"))
(unless (find-package "TUTORIAL_2::SETBOOLRESPONSE")
  (make-package "TUTORIAL_2::SETBOOLRESPONSE"))

(in-package "ROS")





(defclass tutorial_2::SetBoolRequest
  :super ros::object
  :slots (_data ))

(defmethod tutorial_2::SetBoolRequest
  (:init
   (&key
    ((:data __data) nil)
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional (__data :null))
   (if (not (eq __data :null)) (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; bool _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _data
       (if _data (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _data
     (setq _data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass tutorial_2::SetBoolResponse
  :super ros::object
  :slots (_success _message ))

(defmethod tutorial_2::SetBoolResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass tutorial_2::SetBool
  :super ros::object
  :slots ())

(setf (get tutorial_2::SetBool :md5sum-) "09fb03525b03e7ea1fd3992bafd87e16")
(setf (get tutorial_2::SetBool :datatype-) "tutorial_2/SetBool")
(setf (get tutorial_2::SetBool :request) tutorial_2::SetBoolRequest)
(setf (get tutorial_2::SetBool :response) tutorial_2::SetBoolResponse)

(defmethod tutorial_2::SetBoolRequest
  (:response () (instance tutorial_2::SetBoolResponse :init)))

(setf (get tutorial_2::SetBoolRequest :md5sum-) "09fb03525b03e7ea1fd3992bafd87e16")
(setf (get tutorial_2::SetBoolRequest :datatype-) "tutorial_2/SetBoolRequest")
(setf (get tutorial_2::SetBoolRequest :definition-)
      "bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")

(setf (get tutorial_2::SetBoolResponse :md5sum-) "09fb03525b03e7ea1fd3992bafd87e16")
(setf (get tutorial_2::SetBoolResponse :datatype-) "tutorial_2/SetBoolResponse")
(setf (get tutorial_2::SetBoolResponse :definition-)
      "bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")



(provide :tutorial_2/SetBool "09fb03525b03e7ea1fd3992bafd87e16")


