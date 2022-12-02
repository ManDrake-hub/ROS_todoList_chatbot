; Auto-generated. Do not edit!


(cl:in-package rasa_ros-srv)


;//! \htmlinclude Dialogue-request.msg.html

(cl:defclass <Dialogue-request> (roslisp-msg-protocol:ros-message)
  ((input_text
    :reader input_text
    :initarg :input_text
    :type cl:string
    :initform ""))
)

(cl:defclass Dialogue-request (<Dialogue-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Dialogue-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Dialogue-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rasa_ros-srv:<Dialogue-request> is deprecated: use rasa_ros-srv:Dialogue-request instead.")))

(cl:ensure-generic-function 'input_text-val :lambda-list '(m))
(cl:defmethod input_text-val ((m <Dialogue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rasa_ros-srv:input_text-val is deprecated.  Use rasa_ros-srv:input_text instead.")
  (input_text m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Dialogue-request>) ostream)
  "Serializes a message object of type '<Dialogue-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'input_text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'input_text))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Dialogue-request>) istream)
  "Deserializes a message object of type '<Dialogue-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'input_text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'input_text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Dialogue-request>)))
  "Returns string type for a service object of type '<Dialogue-request>"
  "rasa_ros/DialogueRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dialogue-request)))
  "Returns string type for a service object of type 'Dialogue-request"
  "rasa_ros/DialogueRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Dialogue-request>)))
  "Returns md5sum for a message object of type '<Dialogue-request>"
  "af708bc8927c16924fd75aabb46f9abb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Dialogue-request)))
  "Returns md5sum for a message object of type 'Dialogue-request"
  "af708bc8927c16924fd75aabb46f9abb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Dialogue-request>)))
  "Returns full string definition for message of type '<Dialogue-request>"
  (cl:format cl:nil "string input_text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Dialogue-request)))
  "Returns full string definition for message of type 'Dialogue-request"
  (cl:format cl:nil "string input_text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Dialogue-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'input_text))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Dialogue-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Dialogue-request
    (cl:cons ':input_text (input_text msg))
))
;//! \htmlinclude Dialogue-response.msg.html

(cl:defclass <Dialogue-response> (roslisp-msg-protocol:ros-message)
  ((answer
    :reader answer
    :initarg :answer
    :type cl:string
    :initform ""))
)

(cl:defclass Dialogue-response (<Dialogue-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Dialogue-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Dialogue-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rasa_ros-srv:<Dialogue-response> is deprecated: use rasa_ros-srv:Dialogue-response instead.")))

(cl:ensure-generic-function 'answer-val :lambda-list '(m))
(cl:defmethod answer-val ((m <Dialogue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rasa_ros-srv:answer-val is deprecated.  Use rasa_ros-srv:answer instead.")
  (answer m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Dialogue-response>) ostream)
  "Serializes a message object of type '<Dialogue-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'answer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'answer))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Dialogue-response>) istream)
  "Deserializes a message object of type '<Dialogue-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'answer) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'answer) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Dialogue-response>)))
  "Returns string type for a service object of type '<Dialogue-response>"
  "rasa_ros/DialogueResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dialogue-response)))
  "Returns string type for a service object of type 'Dialogue-response"
  "rasa_ros/DialogueResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Dialogue-response>)))
  "Returns md5sum for a message object of type '<Dialogue-response>"
  "af708bc8927c16924fd75aabb46f9abb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Dialogue-response)))
  "Returns md5sum for a message object of type 'Dialogue-response"
  "af708bc8927c16924fd75aabb46f9abb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Dialogue-response>)))
  "Returns full string definition for message of type '<Dialogue-response>"
  (cl:format cl:nil "string answer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Dialogue-response)))
  "Returns full string definition for message of type 'Dialogue-response"
  (cl:format cl:nil "string answer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Dialogue-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'answer))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Dialogue-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Dialogue-response
    (cl:cons ':answer (answer msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Dialogue)))
  'Dialogue-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Dialogue)))
  'Dialogue-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dialogue)))
  "Returns string type for a service object of type '<Dialogue>"
  "rasa_ros/Dialogue")