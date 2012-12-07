; Auto-generated. Do not edit!


(cl:in-package dynamixel_hardware_interface-srv)


;//! \htmlinclude StartController-request.msg.html

(cl:defclass <StartController-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (port
    :reader port
    :initarg :port
    :type cl:string
    :initform ""))
)

(cl:defclass StartController-request (<StartController-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartController-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartController-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_hardware_interface-srv:<StartController-request> is deprecated: use dynamixel_hardware_interface-srv:StartController-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <StartController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_hardware_interface-srv:name-val is deprecated.  Use dynamixel_hardware_interface-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <StartController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_hardware_interface-srv:port-val is deprecated.  Use dynamixel_hardware_interface-srv:port instead.")
  (port m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartController-request>) ostream)
  "Serializes a message object of type '<StartController-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'port))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'port))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartController-request>) istream)
  "Deserializes a message object of type '<StartController-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'port) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'port) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartController-request>)))
  "Returns string type for a service object of type '<StartController-request>"
  "dynamixel_hardware_interface/StartControllerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartController-request)))
  "Returns string type for a service object of type 'StartController-request"
  "dynamixel_hardware_interface/StartControllerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartController-request>)))
  "Returns md5sum for a message object of type '<StartController-request>"
  "ee08ec73d1ee598e2b14b822768462c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartController-request)))
  "Returns md5sum for a message object of type 'StartController-request"
  "ee08ec73d1ee598e2b14b822768462c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartController-request>)))
  "Returns full string definition for message of type '<StartController-request>"
  (cl:format cl:nil "string name~%string port~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartController-request)))
  "Returns full string definition for message of type 'StartController-request"
  (cl:format cl:nil "string name~%string port~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartController-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'port))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartController-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartController-request
    (cl:cons ':name (name msg))
    (cl:cons ':port (port msg))
))
;//! \htmlinclude StartController-response.msg.html

(cl:defclass <StartController-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass StartController-response (<StartController-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartController-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartController-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_hardware_interface-srv:<StartController-response> is deprecated: use dynamixel_hardware_interface-srv:StartController-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <StartController-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_hardware_interface-srv:success-val is deprecated.  Use dynamixel_hardware_interface-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartController-response>) ostream)
  "Serializes a message object of type '<StartController-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartController-response>) istream)
  "Deserializes a message object of type '<StartController-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartController-response>)))
  "Returns string type for a service object of type '<StartController-response>"
  "dynamixel_hardware_interface/StartControllerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartController-response)))
  "Returns string type for a service object of type 'StartController-response"
  "dynamixel_hardware_interface/StartControllerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartController-response>)))
  "Returns md5sum for a message object of type '<StartController-response>"
  "ee08ec73d1ee598e2b14b822768462c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartController-response)))
  "Returns md5sum for a message object of type 'StartController-response"
  "ee08ec73d1ee598e2b14b822768462c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartController-response>)))
  "Returns full string definition for message of type '<StartController-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartController-response)))
  "Returns full string definition for message of type 'StartController-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartController-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartController-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartController-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartController)))
  'StartController-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartController)))
  'StartController-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartController)))
  "Returns string type for a service object of type '<StartController>"
  "dynamixel_hardware_interface/StartController")