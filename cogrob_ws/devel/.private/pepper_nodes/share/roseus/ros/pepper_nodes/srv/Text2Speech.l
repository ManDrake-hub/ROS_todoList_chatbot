;; Auto-generated. Do not edit!


(when (boundp 'pepper_nodes::Text2Speech)
  (if (not (find-package "PEPPER_NODES"))
    (make-package "PEPPER_NODES"))
  (shadow 'Text2Speech (find-package "PEPPER_NODES")))
(unless (find-package "PEPPER_NODES::TEXT2SPEECH")
  (make-package "PEPPER_NODES::TEXT2SPEECH"))
(unless (find-package "PEPPER_NODES::TEXT2SPEECHREQUEST")
  (make-package "PEPPER_NODES::TEXT2SPEECHREQUEST"))
(unless (find-package "PEPPER_NODES::TEXT2SPEECHRESPONSE")
  (make-package "PEPPER_NODES::TEXT2SPEECHRESPONSE"))

(in-package "ROS")





(defclass pepper_nodes::Text2SpeechRequest
  :super ros::object
  :slots (_speech ))

(defmethod pepper_nodes::Text2SpeechRequest
  (:init
   (&key
    ((:speech __speech) "")
    )
   (send-super :init)
   (setq _speech (string __speech))
   self)
  (:speech
   (&optional __speech)
   (if __speech (setq _speech __speech)) _speech)
  (:serialization-length
   ()
   (+
    ;; string _speech
    4 (length _speech)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _speech
       (write-long (length _speech) s) (princ _speech s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _speech
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _speech (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pepper_nodes::Text2SpeechResponse
  :super ros::object
  :slots (_ack ))

(defmethod pepper_nodes::Text2SpeechResponse
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

(defclass pepper_nodes::Text2Speech
  :super ros::object
  :slots ())

(setf (get pepper_nodes::Text2Speech :md5sum-) "8be93dfc13258654eb30fdcda5227e42")
(setf (get pepper_nodes::Text2Speech :datatype-) "pepper_nodes/Text2Speech")
(setf (get pepper_nodes::Text2Speech :request) pepper_nodes::Text2SpeechRequest)
(setf (get pepper_nodes::Text2Speech :response) pepper_nodes::Text2SpeechResponse)

(defmethod pepper_nodes::Text2SpeechRequest
  (:response () (instance pepper_nodes::Text2SpeechResponse :init)))

(setf (get pepper_nodes::Text2SpeechRequest :md5sum-) "8be93dfc13258654eb30fdcda5227e42")
(setf (get pepper_nodes::Text2SpeechRequest :datatype-) "pepper_nodes/Text2SpeechRequest")
(setf (get pepper_nodes::Text2SpeechRequest :definition-)
      "string speech
---
string ack
")

(setf (get pepper_nodes::Text2SpeechResponse :md5sum-) "8be93dfc13258654eb30fdcda5227e42")
(setf (get pepper_nodes::Text2SpeechResponse :datatype-) "pepper_nodes/Text2SpeechResponse")
(setf (get pepper_nodes::Text2SpeechResponse :definition-)
      "string speech
---
string ack
")



(provide :pepper_nodes/Text2Speech "8be93dfc13258654eb30fdcda5227e42")


