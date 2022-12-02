;; Auto-generated. Do not edit!


(when (boundp 'pepper_nodes::ExecuteJS)
  (if (not (find-package "PEPPER_NODES"))
    (make-package "PEPPER_NODES"))
  (shadow 'ExecuteJS (find-package "PEPPER_NODES")))
(unless (find-package "PEPPER_NODES::EXECUTEJS")
  (make-package "PEPPER_NODES::EXECUTEJS"))
(unless (find-package "PEPPER_NODES::EXECUTEJSREQUEST")
  (make-package "PEPPER_NODES::EXECUTEJSREQUEST"))
(unless (find-package "PEPPER_NODES::EXECUTEJSRESPONSE")
  (make-package "PEPPER_NODES::EXECUTEJSRESPONSE"))

(in-package "ROS")





(defclass pepper_nodes::ExecuteJSRequest
  :super ros::object
  :slots (_js ))

(defmethod pepper_nodes::ExecuteJSRequest
  (:init
   (&key
    ((:js __js) "")
    )
   (send-super :init)
   (setq _js (string __js))
   self)
  (:js
   (&optional __js)
   (if __js (setq _js __js)) _js)
  (:serialization-length
   ()
   (+
    ;; string _js
    4 (length _js)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _js
       (write-long (length _js) s) (princ _js s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _js
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _js (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pepper_nodes::ExecuteJSResponse
  :super ros::object
  :slots (_ack ))

(defmethod pepper_nodes::ExecuteJSResponse
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

(defclass pepper_nodes::ExecuteJS
  :super ros::object
  :slots ())

(setf (get pepper_nodes::ExecuteJS :md5sum-) "0bc1212ef5c5830fe8dbd8060c89a89d")
(setf (get pepper_nodes::ExecuteJS :datatype-) "pepper_nodes/ExecuteJS")
(setf (get pepper_nodes::ExecuteJS :request) pepper_nodes::ExecuteJSRequest)
(setf (get pepper_nodes::ExecuteJS :response) pepper_nodes::ExecuteJSResponse)

(defmethod pepper_nodes::ExecuteJSRequest
  (:response () (instance pepper_nodes::ExecuteJSResponse :init)))

(setf (get pepper_nodes::ExecuteJSRequest :md5sum-) "0bc1212ef5c5830fe8dbd8060c89a89d")
(setf (get pepper_nodes::ExecuteJSRequest :datatype-) "pepper_nodes/ExecuteJSRequest")
(setf (get pepper_nodes::ExecuteJSRequest :definition-)
      "string js
---
string ack
")

(setf (get pepper_nodes::ExecuteJSResponse :md5sum-) "0bc1212ef5c5830fe8dbd8060c89a89d")
(setf (get pepper_nodes::ExecuteJSResponse :datatype-) "pepper_nodes/ExecuteJSResponse")
(setf (get pepper_nodes::ExecuteJSResponse :definition-)
      "string js
---
string ack
")



(provide :pepper_nodes/ExecuteJS "0bc1212ef5c5830fe8dbd8060c89a89d")


