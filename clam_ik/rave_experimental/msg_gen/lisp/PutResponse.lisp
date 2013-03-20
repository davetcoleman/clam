; Auto-generated. Do not edit!


(cl:in-package rave_experimental-msg)


;//! \htmlinclude PutResponse.msg.html

(cl:defclass <PutResponse> (roslisp-msg-protocol:ros-message)
  ((object_id
    :reader object_id
    :initarg :object_id
    :type cl:string
    :initform "")
   (errno
    :reader errno
    :initarg :errno
    :type cl:integer
    :initform 0))
)

(cl:defclass PutResponse (<PutResponse>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PutResponse>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PutResponse)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rave_experimental-msg:<PutResponse> is deprecated: use rave_experimental-msg:PutResponse instead.")))

(cl:ensure-generic-function 'object_id-val :lambda-list '(m))
(cl:defmethod object_id-val ((m <PutResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:object_id-val is deprecated.  Use rave_experimental-msg:object_id instead.")
  (object_id m))

(cl:ensure-generic-function 'errno-val :lambda-list '(m))
(cl:defmethod errno-val ((m <PutResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:errno-val is deprecated.  Use rave_experimental-msg:errno instead.")
  (errno m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PutResponse>) ostream)
  "Serializes a message object of type '<PutResponse>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object_id))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'errno)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'errno)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'errno)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'errno)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PutResponse>) istream)
  "Deserializes a message object of type '<PutResponse>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'errno)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'errno)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'errno)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'errno)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PutResponse>)))
  "Returns string type for a message object of type '<PutResponse>"
  "rave_experimental/PutResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PutResponse)))
  "Returns string type for a message object of type 'PutResponse"
  "rave_experimental/PutResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PutResponse>)))
  "Returns md5sum for a message object of type '<PutResponse>"
  "9e7f17b0f2355c303da59bb9ccc1d435")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PutResponse)))
  "Returns md5sum for a message object of type 'PutResponse"
  "9e7f17b0f2355c303da59bb9ccc1d435")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PutResponse>)))
  "Returns full string definition for message of type '<PutResponse>"
  (cl:format cl:nil "string object_id~%uint32 errno~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PutResponse)))
  "Returns full string definition for message of type 'PutResponse"
  (cl:format cl:nil "string object_id~%uint32 errno~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PutResponse>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'object_id))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PutResponse>))
  "Converts a ROS message object to a list"
  (cl:list 'PutResponse
    (cl:cons ':object_id (object_id msg))
    (cl:cons ':errno (errno msg))
))
