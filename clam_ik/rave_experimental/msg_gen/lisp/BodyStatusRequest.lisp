; Auto-generated. Do not edit!


(cl:in-package rave_experimental-msg)


;//! \htmlinclude BodyStatusRequest.msg.html

(cl:defclass <BodyStatusRequest> (roslisp-msg-protocol:ros-message)
  ((object_id
    :reader object_id
    :initarg :object_id
    :type cl:string
    :initform ""))
)

(cl:defclass BodyStatusRequest (<BodyStatusRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BodyStatusRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BodyStatusRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rave_experimental-msg:<BodyStatusRequest> is deprecated: use rave_experimental-msg:BodyStatusRequest instead.")))

(cl:ensure-generic-function 'object_id-val :lambda-list '(m))
(cl:defmethod object_id-val ((m <BodyStatusRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:object_id-val is deprecated.  Use rave_experimental-msg:object_id instead.")
  (object_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BodyStatusRequest>) ostream)
  "Serializes a message object of type '<BodyStatusRequest>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BodyStatusRequest>) istream)
  "Deserializes a message object of type '<BodyStatusRequest>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BodyStatusRequest>)))
  "Returns string type for a message object of type '<BodyStatusRequest>"
  "rave_experimental/BodyStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BodyStatusRequest)))
  "Returns string type for a message object of type 'BodyStatusRequest"
  "rave_experimental/BodyStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BodyStatusRequest>)))
  "Returns md5sum for a message object of type '<BodyStatusRequest>"
  "d552cd960e0a22b19c9db18033cea0eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BodyStatusRequest)))
  "Returns md5sum for a message object of type 'BodyStatusRequest"
  "d552cd960e0a22b19c9db18033cea0eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BodyStatusRequest>)))
  "Returns full string definition for message of type '<BodyStatusRequest>"
  (cl:format cl:nil "string object_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BodyStatusRequest)))
  "Returns full string definition for message of type 'BodyStatusRequest"
  (cl:format cl:nil "string object_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BodyStatusRequest>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'object_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BodyStatusRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'BodyStatusRequest
    (cl:cons ':object_id (object_id msg))
))
