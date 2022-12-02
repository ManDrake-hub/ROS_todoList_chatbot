;; Auto-generated. Do not edit!


(when (boundp 'pepper_nodes::Rest)
  (if (not (find-package "PEPPER_NODES"))
    (make-package "PEPPER_NODES"))
  (shadow 'Rest (find-package "PEPPER_NODES")))
(unless (find-package "PEPPER_NODES::REST")
  (make-package "PEPPER_NODES::REST"))
(unless (find-package "PEPPER_NODES::RESTREQUEST")
  (make-package "PEPPER_NODES::RESTREQUEST"))
(unless (find-package "PEPPER_NODES::RESTRESPONSE")
  (make-package "PEPPER_NODES::RESTRESPONSE"))

(in-package "ROS")





(defclass pepper_nodes::RestRequest
  :super ros::object
  :slots ())

(defmethod pepper_nodes::RestRequest
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

(defclass pepper_nodes::RestResponse
  :super ros::object
  :slots (_ack ))

(defmethod pepper_nodes::RestResponse
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

(defclass pepper_nodes::Rest
  :super ros::object
  :slots ())

(setf (get pepper_nodes::Rest :md5sum-) "b6a73f722475d64a28238118997ef482")
(setf (get pepper_nodes::Rest :datatype-) "pepper_nodes/Rest")
(setf (get pepper_nodes::Rest :request) pepper_nodes::RestRequest)
(setf (get pepper_nodes::Rest :response) pepper_nodes::RestResponse)

(defmethod pepper_nodes::RestRequest
  (:response () (instance pepper_nodes::RestResponse :init)))

(setf (get pepper_nodes::RestRequest :md5sum-) "b6a73f722475d64a28238118997ef482")
(setf (get pepper_nodes::RestRequest :datatype-) "pepper_nodes/RestRequest")
(setf (get pepper_nodes::RestRequest :definition-)
      "---
string ack
")

(setf (get pepper_nodes::RestResponse :md5sum-) "b6a73f722475d64a28238118997ef482")
(setf (get pepper_nodes::RestResponse :datatype-) "pepper_nodes/RestResponse")
(setf (get pepper_nodes::RestResponse :definition-)
      "---
string ack
")



(provide :pepper_nodes/Rest "b6a73f722475d64a28238118997ef482")


