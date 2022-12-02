; Auto-generated. Do not edit!


(cl:in-package pepper_nodes-srv)


;//! \htmlinclude WakeUp-request.msg.html

(cl:defclass <WakeUp-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WakeUp-request (<WakeUp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WakeUp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WakeUp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pepper_nodes-srv:<WakeUp-request> is deprecated: use pepper_nodes-srv:WakeUp-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WakeUp-request>) ostream)
  "Serializes a message object of type '<WakeUp-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WakeUp-request>) istream)
  "Deserializes a message object of type '<WakeUp-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WakeUp-request>)))
  "Returns string type for a service object of type '<WakeUp-request>"
  "pepper_nodes/WakeUpRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WakeUp-request)))
  "Returns string type for a service object of type 'WakeUp-request"
  "pepper_nodes/WakeUpRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WakeUp-request>)))
  "Returns md5sum for a message object of type '<WakeUp-request>"
  "b6a73f722475d64a28238118997ef482")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WakeUp-request)))
  "Returns md5sum for a message object of type 'WakeUp-request"
  "b6a73f722475d64a28238118997ef482")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WakeUp-request>)))
  "Returns full string definition for message of type '<WakeUp-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WakeUp-request)))
  "Returns full string definition for message of type 'WakeUp-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WakeUp-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WakeUp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WakeUp-request
))
;//! \htmlinclude WakeUp-response.msg.html

(cl:defclass <WakeUp-response> (roslisp-msg-protocol:ros-message)
  ((ack
    :reader ack
    :initarg :ack
    :type cl:string
    :initform ""))
)

(cl:defclass WakeUp-response (<WakeUp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WakeUp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WakeUp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pepper_nodes-srv:<WakeUp-response> is deprecated: use pepper_nodes-srv:WakeUp-response instead.")))

(cl:ensure-generic-function 'ack-val :lambda-list '(m))
(cl:defmethod ack-val ((m <WakeUp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pepper_nodes-srv:ack-val is deprecated.  Use pepper_nodes-srv:ack instead.")
  (ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WakeUp-response>) ostream)
  "Serializes a message object of type '<WakeUp-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ack))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ack))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WakeUp-response>) istream)
  "Deserializes a message object of type '<WakeUp-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WakeUp-response>)))
  "Returns string type for a service object of type '<WakeUp-response>"
  "pepper_nodes/WakeUpResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WakeUp-response)))
  "Returns string type for a service object of type 'WakeUp-response"
  "pepper_nodes/WakeUpResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WakeUp-response>)))
  "Returns md5sum for a message object of type '<WakeUp-response>"
  "b6a73f722475d64a28238118997ef482")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WakeUp-response)))
  "Returns md5sum for a message object of type 'WakeUp-response"
  "b6a73f722475d64a28238118997ef482")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WakeUp-response>)))
  "Returns full string definition for message of type '<WakeUp-response>"
  (cl:format cl:nil "string ack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WakeUp-response)))
  "Returns full string definition for message of type 'WakeUp-response"
  (cl:format cl:nil "string ack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WakeUp-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'ack))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WakeUp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WakeUp-response
    (cl:cons ':ack (ack msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WakeUp)))
  'WakeUp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WakeUp)))
  'WakeUp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WakeUp)))
  "Returns string type for a service object of type '<WakeUp>"
  "pepper_nodes/WakeUp")