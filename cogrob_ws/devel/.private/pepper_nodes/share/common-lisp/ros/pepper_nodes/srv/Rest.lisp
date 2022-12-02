; Auto-generated. Do not edit!


(cl:in-package pepper_nodes-srv)


;//! \htmlinclude Rest-request.msg.html

(cl:defclass <Rest-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Rest-request (<Rest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Rest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Rest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pepper_nodes-srv:<Rest-request> is deprecated: use pepper_nodes-srv:Rest-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Rest-request>) ostream)
  "Serializes a message object of type '<Rest-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Rest-request>) istream)
  "Deserializes a message object of type '<Rest-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Rest-request>)))
  "Returns string type for a service object of type '<Rest-request>"
  "pepper_nodes/RestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Rest-request)))
  "Returns string type for a service object of type 'Rest-request"
  "pepper_nodes/RestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Rest-request>)))
  "Returns md5sum for a message object of type '<Rest-request>"
  "b6a73f722475d64a28238118997ef482")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Rest-request)))
  "Returns md5sum for a message object of type 'Rest-request"
  "b6a73f722475d64a28238118997ef482")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Rest-request>)))
  "Returns full string definition for message of type '<Rest-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Rest-request)))
  "Returns full string definition for message of type 'Rest-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Rest-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Rest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Rest-request
))
;//! \htmlinclude Rest-response.msg.html

(cl:defclass <Rest-response> (roslisp-msg-protocol:ros-message)
  ((ack
    :reader ack
    :initarg :ack
    :type cl:string
    :initform ""))
)

(cl:defclass Rest-response (<Rest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Rest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Rest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pepper_nodes-srv:<Rest-response> is deprecated: use pepper_nodes-srv:Rest-response instead.")))

(cl:ensure-generic-function 'ack-val :lambda-list '(m))
(cl:defmethod ack-val ((m <Rest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pepper_nodes-srv:ack-val is deprecated.  Use pepper_nodes-srv:ack instead.")
  (ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Rest-response>) ostream)
  "Serializes a message object of type '<Rest-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ack))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ack))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Rest-response>) istream)
  "Deserializes a message object of type '<Rest-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Rest-response>)))
  "Returns string type for a service object of type '<Rest-response>"
  "pepper_nodes/RestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Rest-response)))
  "Returns string type for a service object of type 'Rest-response"
  "pepper_nodes/RestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Rest-response>)))
  "Returns md5sum for a message object of type '<Rest-response>"
  "b6a73f722475d64a28238118997ef482")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Rest-response)))
  "Returns md5sum for a message object of type 'Rest-response"
  "b6a73f722475d64a28238118997ef482")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Rest-response>)))
  "Returns full string definition for message of type '<Rest-response>"
  (cl:format cl:nil "string ack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Rest-response)))
  "Returns full string definition for message of type 'Rest-response"
  (cl:format cl:nil "string ack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Rest-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'ack))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Rest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Rest-response
    (cl:cons ':ack (ack msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Rest)))
  'Rest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Rest)))
  'Rest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Rest)))
  "Returns string type for a service object of type '<Rest>"
  "pepper_nodes/Rest")