;; Auto-generated. Do not edit!


(when (boundp 'rasa_ros::Dialogue)
  (if (not (find-package "RASA_ROS"))
    (make-package "RASA_ROS"))
  (shadow 'Dialogue (find-package "RASA_ROS")))
(unless (find-package "RASA_ROS::DIALOGUE")
  (make-package "RASA_ROS::DIALOGUE"))
(unless (find-package "RASA_ROS::DIALOGUEREQUEST")
  (make-package "RASA_ROS::DIALOGUEREQUEST"))
(unless (find-package "RASA_ROS::DIALOGUERESPONSE")
  (make-package "RASA_ROS::DIALOGUERESPONSE"))

(in-package "ROS")





(defclass rasa_ros::DialogueRequest
  :super ros::object
  :slots (_input_text ))

(defmethod rasa_ros::DialogueRequest
  (:init
   (&key
    ((:input_text __input_text) "")
    )
   (send-super :init)
   (setq _input_text (string __input_text))
   self)
  (:input_text
   (&optional __input_text)
   (if __input_text (setq _input_text __input_text)) _input_text)
  (:serialization-length
   ()
   (+
    ;; string _input_text
    4 (length _input_text)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _input_text
       (write-long (length _input_text) s) (princ _input_text s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _input_text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _input_text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rasa_ros::DialogueResponse
  :super ros::object
  :slots (_answer ))

(defmethod rasa_ros::DialogueResponse
  (:init
   (&key
    ((:answer __answer) "")
    )
   (send-super :init)
   (setq _answer (string __answer))
   self)
  (:answer
   (&optional __answer)
   (if __answer (setq _answer __answer)) _answer)
  (:serialization-length
   ()
   (+
    ;; string _answer
    4 (length _answer)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _answer
       (write-long (length _answer) s) (princ _answer s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _answer
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _answer (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rasa_ros::Dialogue
  :super ros::object
  :slots ())

(setf (get rasa_ros::Dialogue :md5sum-) "af708bc8927c16924fd75aabb46f9abb")
(setf (get rasa_ros::Dialogue :datatype-) "rasa_ros/Dialogue")
(setf (get rasa_ros::Dialogue :request) rasa_ros::DialogueRequest)
(setf (get rasa_ros::Dialogue :response) rasa_ros::DialogueResponse)

(defmethod rasa_ros::DialogueRequest
  (:response () (instance rasa_ros::DialogueResponse :init)))

(setf (get rasa_ros::DialogueRequest :md5sum-) "af708bc8927c16924fd75aabb46f9abb")
(setf (get rasa_ros::DialogueRequest :datatype-) "rasa_ros/DialogueRequest")
(setf (get rasa_ros::DialogueRequest :definition-)
      "string input_text
---
string answer
")

(setf (get rasa_ros::DialogueResponse :md5sum-) "af708bc8927c16924fd75aabb46f9abb")
(setf (get rasa_ros::DialogueResponse :datatype-) "rasa_ros/DialogueResponse")
(setf (get rasa_ros::DialogueResponse :definition-)
      "string input_text
---
string answer
")



(provide :rasa_ros/Dialogue "af708bc8927c16924fd75aabb46f9abb")


