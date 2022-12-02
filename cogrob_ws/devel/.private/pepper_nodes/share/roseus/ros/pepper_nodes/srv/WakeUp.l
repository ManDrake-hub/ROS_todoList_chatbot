;; Auto-generated. Do not edit!


(when (boundp 'pepper_nodes::WakeUp)
  (if (not (find-package "PEPPER_NODES"))
    (make-package "PEPPER_NODES"))
  (shadow 'WakeUp (find-package "PEPPER_NODES")))
(unless (find-package "PEPPER_NODES::WAKEUP")
  (make-package "PEPPER_NODES::WAKEUP"))
(unless (find-package "PEPPER_NODES::WAKEUPREQUEST")
  (make-package "PEPPER_NODES::WAKEUPREQUEST"))
(unless (find-package "PEPPER_NODES::WAKEUPRESPONSE")
  (make-package "PEPPER_NODES::WAKEUPRESPONSE"))

(in-package "ROS")





(defclass pepper_nodes::WakeUpRequest
  :super ros::object
  :slots ())

(defmethod pepper_nodes::WakeUpRequest
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

(defclass pepper_nodes::WakeUpResponse
  :super ros::object
  :slots (_ack ))

(defmethod pepper_nodes::WakeUpResponse
  (:init
   (&key
    ((:ack __ack) "")
    )
   (send-super :init)
   (setq _ack (string __ack))
   self)
  (:ack
   (&optional __ack)
   (if __ack (setq _ack __ack)) _ack)
  (:serialization-length
   ()
   (+
    ;; string _ack
    4 (length _ack)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _ack
       (write-long (length _ack) s) (princ _ack s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _ack
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ack (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pepper_nodes::WakeUp
  :super ros::object
  :slots ())

(setf (get pepper_nodes::WakeUp :md5sum-) "b6a73f722475d64a28238118997ef482")
(setf (get pepper_nodes::WakeUp :datatype-) "pepper_nodes/WakeUp")
(setf (get pepper_nodes::WakeUp :request) pepper_nodes::WakeUpRequest)
(setf (get pepper_nodes::WakeUp :response) pepper_nodes::WakeUpResponse)

(defmethod pepper_nodes::WakeUpRequest
  (:response () (instance pepper_nodes::WakeUpResponse :init)))

(setf (get pepper_nodes::WakeUpRequest :md5sum-) "b6a73f722475d64a28238118997ef482")
(setf (get pepper_nodes::WakeUpRequest :datatype-) "pepper_nodes/WakeUpRequest")
(setf (get pepper_nodes::WakeUpRequest :definition-)
      "---
string ack
")

(setf (get pepper_nodes::WakeUpResponse :md5sum-) "b6a73f722475d64a28238118997ef482")
(setf (get pepper_nodes::WakeUpResponse :datatype-) "pepper_nodes/WakeUpResponse")
(setf (get pepper_nodes::WakeUpResponse :definition-)
      "---
string ack
")



(provide :pepper_nodes/WakeUp "b6a73f722475d64a28238118997ef482")


