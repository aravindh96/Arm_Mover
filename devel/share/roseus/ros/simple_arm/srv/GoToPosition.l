;; Auto-generated. Do not edit!


(when (boundp 'simple_arm::GoToPosition)
  (if (not (find-package "SIMPLE_ARM"))
    (make-package "SIMPLE_ARM"))
  (shadow 'GoToPosition (find-package "SIMPLE_ARM")))
(unless (find-package "SIMPLE_ARM::GOTOPOSITION")
  (make-package "SIMPLE_ARM::GOTOPOSITION"))
(unless (find-package "SIMPLE_ARM::GOTOPOSITIONREQUEST")
  (make-package "SIMPLE_ARM::GOTOPOSITIONREQUEST"))
(unless (find-package "SIMPLE_ARM::GOTOPOSITIONRESPONSE")
  (make-package "SIMPLE_ARM::GOTOPOSITIONRESPONSE"))

(in-package "ROS")





(defclass simple_arm::GoToPositionRequest
  :super ros::object
  :slots (_joint_1_position _joint_2_position ))

(defmethod simple_arm::GoToPositionRequest
  (:init
   (&key
    ((:joint_1_position __joint_1_position) 0.0)
    ((:joint_2_position __joint_2_position) 0.0)
    )
   (send-super :init)
   (setq _joint_1_position (float __joint_1_position))
   (setq _joint_2_position (float __joint_2_position))
   self)
  (:joint_1_position
   (&optional __joint_1_position)
   (if __joint_1_position (setq _joint_1_position __joint_1_position)) _joint_1_position)
  (:joint_2_position
   (&optional __joint_2_position)
   (if __joint_2_position (setq _joint_2_position __joint_2_position)) _joint_2_position)
  (:serialization-length
   ()
   (+
    ;; float64 _joint_1_position
    8
    ;; float64 _joint_2_position
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _joint_1_position
       (sys::poke _joint_1_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _joint_2_position
       (sys::poke _joint_2_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _joint_1_position
     (setq _joint_1_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _joint_2_position
     (setq _joint_2_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass simple_arm::GoToPositionResponse
  :super ros::object
  :slots (_msg_feedback ))

(defmethod simple_arm::GoToPositionResponse
  (:init
   (&key
    ((:msg_feedback __msg_feedback) "")
    )
   (send-super :init)
   (setq _msg_feedback (string __msg_feedback))
   self)
  (:msg_feedback
   (&optional __msg_feedback)
   (if __msg_feedback (setq _msg_feedback __msg_feedback)) _msg_feedback)
  (:serialization-length
   ()
   (+
    ;; string _msg_feedback
    4 (length _msg_feedback)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _msg_feedback
       (write-long (length _msg_feedback) s) (princ _msg_feedback s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _msg_feedback
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg_feedback (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass simple_arm::GoToPosition
  :super ros::object
  :slots ())

(setf (get simple_arm::GoToPosition :md5sum-) "2cfecef4f6d375467fcd431fdce02c32")
(setf (get simple_arm::GoToPosition :datatype-) "simple_arm/GoToPosition")
(setf (get simple_arm::GoToPosition :request) simple_arm::GoToPositionRequest)
(setf (get simple_arm::GoToPosition :response) simple_arm::GoToPositionResponse)

(defmethod simple_arm::GoToPositionRequest
  (:response () (instance simple_arm::GoToPositionResponse :init)))

(setf (get simple_arm::GoToPositionRequest :md5sum-) "2cfecef4f6d375467fcd431fdce02c32")
(setf (get simple_arm::GoToPositionRequest :datatype-) "simple_arm/GoToPositionRequest")
(setf (get simple_arm::GoToPositionRequest :definition-)
      "float64 joint_1_position
float64 joint_2_position
---
string msg_feedback

")

(setf (get simple_arm::GoToPositionResponse :md5sum-) "2cfecef4f6d375467fcd431fdce02c32")
(setf (get simple_arm::GoToPositionResponse :datatype-) "simple_arm/GoToPositionResponse")
(setf (get simple_arm::GoToPositionResponse :definition-)
      "float64 joint_1_position
float64 joint_2_position
---
string msg_feedback

")



(provide :simple_arm/GoToPosition "2cfecef4f6d375467fcd431fdce02c32")


