; Auto-generated. Do not edit!


(cl:in-package pepper_nodes-srv)


;//! \htmlinclude LoadUrl-request.msg.html

(cl:defclass <LoadUrl-request> (roslisp-msg-protocol:ros-message)
  ((url
    :reader url
    :initarg :url
    :type cl:string
    :initform ""))
)

(cl:defclass LoadUrl-request (<LoadUrl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadUrl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadUrl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pepper_nodes-srv:<LoadUrl-request> is deprecated: use pepper_nodes-srv:LoadUrl-request instead.")))

(cl:ensure-generic-function 'url-val :lambda-list '(m))
(cl:defmethod url-val ((m <LoadUrl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pepper_nodes-srv:url-val is deprecated.  Use pepper_nodes-srv:url instead.")
  (url m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadUrl-request>) ostream)
  "Serializes a message object of type '<LoadUrl-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'url))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'url))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadUrl-request>) istream)
  "Deserializes a message object of type '<LoadUrl-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'url) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'url) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadUrl-request>)))
  "Returns string type for a service object of type '<LoadUrl-request>"
  "pepper_nodes/LoadUrlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadUrl-request)))
  "Returns string type for a service object of type 'LoadUrl-request"
  "pepper_nodes/LoadUrlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadUrl-request>)))
  "Returns md5sum for a message object of type '<LoadUrl-request>"
  "5562f0f326dc984bc777bae8e1589603")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadUrl-request)))
  "Returns md5sum for a message object of type 'LoadUrl-request"
  "5562f0f326dc984bc777bae8e1589603")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadUrl-request>)))
  "Returns full string definition for message of type '<LoadUrl-request>"
  (cl:format cl:nil "string url~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadUrl-request)))
  "Returns full string definition for message of type 'LoadUrl-request"
  (cl:format cl:nil "string url~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadUrl-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'url))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadUrl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadUrl-request
    (cl:cons ':url (url msg))
))
;//! \htmlinclude LoadUrl-response.msg.html

(cl:defclass <LoadUrl-response> (roslisp-msg-protocol:ros-message)
  ((ack
    :reader ack
    :initarg :ack
    :type cl:string
    :initform ""))
)

(cl:defclass LoadUrl-response (<LoadUrl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadUrl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadUrl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pepper_nodes-srv:<LoadUrl-response> is deprecated: use pepper_nodes-srv:LoadUrl-response instead.")))

(cl:ensure-generic-function 'ack-val :lambda-list '(m))
(cl:defmethod ack-val ((m <LoadUrl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pepper_nodes-srv:ack-val is deprecated.  Use pepper_nodes-srv:ack instead.")
  (ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadUrl-response>) ostream)
  "Serializes a message object of type '<LoadUrl-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ack))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ack))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadUrl-response>) istream)
  "Deserializes a message object of type '<LoadUrl-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadUrl-response>)))
  "Returns string type for a service object of type '<LoadUrl-response>"
  "pepper_nodes/LoadUrlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadUrl-response)))
  "Returns string type for a service object of type 'LoadUrl-response"
  "pepper_nodes/LoadUrlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadUrl-response>)))
  "Returns md5sum for a message object of type '<LoadUrl-response>"
  "5562f0f326dc984bc777bae8e1589603")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadUrl-response)))
  "Returns md5sum for a message object of type 'LoadUrl-response"
  "5562f0f326dc984bc777bae8e1589603")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadUrl-response>)))
  "Returns full string definition for message of type '<LoadUrl-response>"
  (cl:format cl:nil "string ack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadUrl-response)))
  "Returns full string definition for message of type 'LoadUrl-response"
  (cl:format cl:nil "string ack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadUrl-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'ack))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadUrl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadUrl-response
    (cl:cons ':ack (ack msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LoadUrl)))
  'LoadUrl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LoadUrl)))
  'LoadUrl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadUrl)))
  "Returns string type for a service object of type '<LoadUrl>"
  "pepper_nodes/LoadUrl")