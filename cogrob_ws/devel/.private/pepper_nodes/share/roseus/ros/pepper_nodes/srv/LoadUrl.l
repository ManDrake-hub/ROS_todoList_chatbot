;; Auto-generated. Do not edit!


(when (boundp 'pepper_nodes::LoadUrl)
  (if (not (find-package "PEPPER_NODES"))
    (make-package "PEPPER_NODES"))
  (shadow 'LoadUrl (find-package "PEPPER_NODES")))
(unless (find-package "PEPPER_NODES::LOADURL")
  (make-package "PEPPER_NODES::LOADURL"))
(unless (find-package "PEPPER_NODES::LOADURLREQUEST")
  (make-package "PEPPER_NODES::LOADURLREQUEST"))
(unless (find-package "PEPPER_NODES::LOADURLRESPONSE")
  (make-package "PEPPER_NODES::LOADURLRESPONSE"))

(in-package "ROS")





(defclass pepper_nodes::LoadUrlRequest
  :super ros::object
  :slots (_url ))

(defmethod pepper_nodes::LoadUrlRequest
  (:init
   (&key
    ((:url __url) "")
    )
   (send-super :init)
   (setq _url (string __url))
   self)
  (:url
   (&optional __url)
   (if __url (setq _url __url)) _url)
  (:serialization-length
   ()
   (+
    ;; string _url
    4 (length _url)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _url
       (write-long (length _url) s) (princ _url s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _url
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _url (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pepper_nodes::LoadUrlResponse
  :super ros::object
  :slots (_ack ))

(defmethod pepper_nodes::LoadUrlResponse
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

(defclass pepper_nodes::LoadUrl
  :super ros::object
  :slots ())

(setf (get pepper_nodes::LoadUrl :md5sum-) "5562f0f326dc984bc777bae8e1589603")
(setf (get pepper_nodes::LoadUrl :datatype-) "pepper_nodes/LoadUrl")
(setf (get pepper_nodes::LoadUrl :request) pepper_nodes::LoadUrlRequest)
(setf (get pepper_nodes::LoadUrl :response) pepper_nodes::LoadUrlResponse)

(defmethod pepper_nodes::LoadUrlRequest
  (:response () (instance pepper_nodes::LoadUrlResponse :init)))

(setf (get pepper_nodes::LoadUrlRequest :md5sum-) "5562f0f326dc984bc777bae8e1589603")
(setf (get pepper_nodes::LoadUrlRequest :datatype-) "pepper_nodes/LoadUrlRequest")
(setf (get pepper_nodes::LoadUrlRequest :definition-)
      "string url
---
string ack
")

(setf (get pepper_nodes::LoadUrlResponse :md5sum-) "5562f0f326dc984bc777bae8e1589603")
(setf (get pepper_nodes::LoadUrlResponse :datatype-) "pepper_nodes/LoadUrlResponse")
(setf (get pepper_nodes::LoadUrlResponse :definition-)
      "string url
---
string ack
")



(provide :pepper_nodes/LoadUrl "5562f0f326dc984bc777bae8e1589603")


