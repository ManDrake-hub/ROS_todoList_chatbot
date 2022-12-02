; Auto-generated. Do not edit!


(cl:in-package pepper_nodes-srv)


;//! \htmlinclude ExecuteJS-request.msg.html

(cl:defclass <ExecuteJS-request> (roslisp-msg-protocol:ros-message)
  ((js
    :reader js
    :initarg :js
    :type cl:string
    :initform ""))
)

(cl:defclass ExecuteJS-request (<ExecuteJS-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteJS-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteJS-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pepper_nodes-srv:<ExecuteJS-request> is deprecated: use pepper_nodes-srv:ExecuteJS-request instead.")))

(cl:ensure-generic-function 'js-val :lambda-list '(m))
(cl:defmethod js-val ((m <ExecuteJS-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pepper_nodes-srv:js-val is deprecated.  Use pepper_nodes-srv:js instead.")
  (js m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteJS-request>) ostream)
  "Serializes a message object of type '<ExecuteJS-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'js))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'js))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteJS-request>) istream)
  "Deserializes a message object of type '<ExecuteJS-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'js) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'js) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteJS-request>)))
  "Returns string type for a service object of type '<ExecuteJS-request>"
  "pepper_nodes/ExecuteJSRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteJS-request)))
  "Returns string type for a service object of type 'ExecuteJS-request"
  "pepper_nodes/ExecuteJSRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteJS-request>)))
  "Returns md5sum for a message object of type '<ExecuteJS-request>"
  "0bc1212ef5c5830fe8dbd8060c89a89d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteJS-request)))
  "Returns md5sum for a message object of type 'ExecuteJS-request"
  "0bc1212ef5c5830fe8dbd8060c89a89d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteJS-request>)))
  "Returns full string definition for message of type '<ExecuteJS-request>"
  (cl:format cl:nil "string js~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteJS-request)))
  "Returns full string definition for message of type 'ExecuteJS-request"
  (cl:format cl:nil "string js~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteJS-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'js))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteJS-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteJS-request
    (cl:cons ':js (js msg))
))
;//! \htmlinclude ExecuteJS-response.msg.html

(cl:defclass <ExecuteJS-response> (roslisp-msg-protocol:ros-message)
  ((ack
    :reader ack
    :initarg :ack
    :type cl:string
    :initform ""))
)

(cl:defclass ExecuteJS-response (<ExecuteJS-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteJS-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteJS-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pepper_nodes-srv:<ExecuteJS-response> is deprecated: use pepper_nodes-srv:ExecuteJS-response instead.")))

(cl:ensure-generic-function 'ack-val :lambda-list '(m))
(cl:defmethod ack-val ((m <ExecuteJS-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pepper_nodes-srv:ack-val is deprecated.  Use pepper_nodes-srv:ack instead.")
  (ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteJS-response>) ostream)
  "Serializes a message object of type '<ExecuteJS-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ack))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ack))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteJS-response>) istream)
  "Deserializes a message object of type '<ExecuteJS-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteJS-response>)))
  "Returns string type for a service object of type '<ExecuteJS-response>"
  "pepper_nodes/ExecuteJSResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteJS-response)))
  "Returns string type for a service object of type 'ExecuteJS-response"
  "pepper_nodes/ExecuteJSResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteJS-response>)))
  "Returns md5sum for a message object of type '<ExecuteJS-response>"
  "0bc1212ef5c5830fe8dbd8060c89a89d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteJS-response)))
  "Returns md5sum for a message object of type 'ExecuteJS-response"
  "0bc1212ef5c5830fe8dbd8060c89a89d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteJS-response>)))
  "Returns full string definition for message of type '<ExecuteJS-response>"
  (cl:format cl:nil "string ack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteJS-response)))
  "Returns full string definition for message of type 'ExecuteJS-response"
  (cl:format cl:nil "string ack~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteJS-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'ack))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteJS-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteJS-response
    (cl:cons ':ack (ack msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ExecuteJS)))
  'ExecuteJS-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ExecuteJS)))
  'ExecuteJS-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteJS)))
  "Returns string type for a service object of type '<ExecuteJS>"
  "pepper_nodes/ExecuteJS")