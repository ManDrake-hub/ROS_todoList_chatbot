; Auto-generated. Do not edit!


(cl:in-package pepper_nodes-srv)


;//! \htmlinclude Text2Speech-request.msg.html

(cl:defclass <Text2Speech-request> (roslisp-msg-protocol:ros-message)
  ((speech
    :reader speech
    :initarg :speech
    :type cl:string
    :initform ""))
)

(cl:defclass Text2Speech-request (<Text2Speech-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Text2Speech-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Text2Speech-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pepper_nodes-srv:<Text2Speech-request> is deprecated: use pepper_nodes-srv:Text2Speech-request instead.")))

(cl:ensure-generic-function 'speech-val :lambda-list '(m))
(cl:defmethod speech-val ((m <Text2Speech-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pepper_nodes-srv:speech-val is deprecated.  Use pepper_nodes-srv:speech instead.")
  (speech m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Text2Speech-request>) ostream)
  "Serializes a message object of type '<Text2Speech-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'speech))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'speech))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Text2Speech-request>) istream)
  "Deserializes a message object of type '<Text2Speech-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speech) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'speech) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Text2Speech-request>)))
  "Returns string type for a service object of type '<Text2Speech-request>"
  "pepper_nodes/Text2SpeechRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Text2Speech-request)))
  "Returns string type for a service object of type 'Text2Speech-request"
  "pepper_nodes/Text2SpeechRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Text2Speech-request>)))
  "Returns md5sum for a message object of type '<Text2Speech-request>"
  "8be93dfc13258654eb30fdcda5227e42")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Text2Speech-request)))
  "Returns md5sum for a message object of type 'Text2Speech-request"
  "8be93dfc13258654eb30fdcda5227e42")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Text2Speech-request>)))
  "Returns full string definition for message of type '<Text2Speech-request>"
  (cl:format cl:nil "string speech~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Text2Speech-request)))
  "Returns full string definition for message of type 'Text2Speech-request"
  (cl:format cl:nil "string speech~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Text2Speech-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'speech))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Text2Speech-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Text2Speech-request
    (cl:cons ':speech (speech msg))
))
;//! \htmlinclude Text2Speech-response.msg.html

(cl:defclass <Text2Speech-response> (roslisp-msg-protocol:ros-message)
  ((ack
    :reader ack
    :initarg :ack
    :type cl:string
    :initform ""))
)

(cl:defclass Text2Speech-response (<Text2Speech-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Text2Speech-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Text2Speech-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pepper_nodes-srv:<Text2Speech-response> is deprecated: use pepper_nodes-srv:Text2Speech-response instead.")))

(cl:ensure-generic-function 'ack-val :lambda-list '(m))
(cl:defmethod ack-val ((m <Text2Speech-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pepper_nodes-srv:ack-val is deprecated.  Use pepper_nodes-srv:ack instead.")
  (ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Text2Speech-response>) ostream)
  "Serializes a message object of type '<Text2Speech-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ack))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ack))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Text2Speech-response>) istream)
  "Deserializes a message object of type '<Text2Speech-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ack) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ack) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Text2Speech-response>)))
  "Returns string type for a service object of type '<Text2Speech-response>"
  "pepper_nodes/Text2SpeechResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Text2Speech-response)))
  "Returns string type for a service object of type 'Text2Speech-response"
  "pepper_nodes/Text2SpeechResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Text2Speech-response>)))
  "Returns md5sum for a message object of type '<Text2Speech-response>"
  "8be93dfc13258654eb30fdcda5227e42")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Text2Speech-response)))
  "Returns md5sum for a message object of type 'Text2Speech-response"
  "8be93dfc13258654eb30fdcda5227e42")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Text2Speech-response>)))
  "Returns full string definition for message of type '<Text2Speech-response>"
  (cl:format cl:nil "string ack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Text2Speech-response)))
  "Returns full string definition for message of type 'Text2Speech-response"
  (cl:format cl:nil "string ack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Text2Speech-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'ack))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Text2Speech-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Text2Speech-response
    (cl:cons ':ack (ack msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Text2Speech)))
  'Text2Speech-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Text2Speech)))
  'Text2Speech-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Text2Speech)))
  "Returns string type for a service object of type '<Text2Speech>"
  "pepper_nodes/Text2Speech")