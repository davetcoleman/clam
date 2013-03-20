; Auto-generated. Do not edit!


(cl:in-package rave_experimental-msg)


;//! \htmlinclude BodyStatus.msg.html

(cl:defclass <BodyStatus> (roslisp-msg-protocol:ros-message)
  ((object_id
    :reader object_id
    :initarg :object_id
    :type cl:string
    :initform "")
   (picked_up
    :reader picked_up
    :initarg :picked_up
    :type cl:boolean
    :initform cl:nil)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (errno
    :reader errno
    :initarg :errno
    :type cl:float
    :initform 0.0))
)

(cl:defclass BodyStatus (<BodyStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BodyStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BodyStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rave_experimental-msg:<BodyStatus> is deprecated: use rave_experimental-msg:BodyStatus instead.")))

(cl:ensure-generic-function 'object_id-val :lambda-list '(m))
(cl:defmethod object_id-val ((m <BodyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:object_id-val is deprecated.  Use rave_experimental-msg:object_id instead.")
  (object_id m))

(cl:ensure-generic-function 'picked_up-val :lambda-list '(m))
(cl:defmethod picked_up-val ((m <BodyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:picked_up-val is deprecated.  Use rave_experimental-msg:picked_up instead.")
  (picked_up m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <BodyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:x-val is deprecated.  Use rave_experimental-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <BodyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:y-val is deprecated.  Use rave_experimental-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <BodyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:z-val is deprecated.  Use rave_experimental-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'errno-val :lambda-list '(m))
(cl:defmethod errno-val ((m <BodyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:errno-val is deprecated.  Use rave_experimental-msg:errno instead.")
  (errno m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BodyStatus>) ostream)
  "Serializes a message object of type '<BodyStatus>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object_id))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'picked_up) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'errno))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BodyStatus>) istream)
  "Deserializes a message object of type '<BodyStatus>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'picked_up) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'errno) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BodyStatus>)))
  "Returns string type for a message object of type '<BodyStatus>"
  "rave_experimental/BodyStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BodyStatus)))
  "Returns string type for a message object of type 'BodyStatus"
  "rave_experimental/BodyStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BodyStatus>)))
  "Returns md5sum for a message object of type '<BodyStatus>"
  "9d67177aea4c214587c878f7ce226d1f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BodyStatus)))
  "Returns md5sum for a message object of type 'BodyStatus"
  "9d67177aea4c214587c878f7ce226d1f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BodyStatus>)))
  "Returns full string definition for message of type '<BodyStatus>"
  (cl:format cl:nil "string object_id~%bool picked_up~%float32 x~%float32 y~%float32 z~%float32 errno~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BodyStatus)))
  "Returns full string definition for message of type 'BodyStatus"
  (cl:format cl:nil "string object_id~%bool picked_up~%float32 x~%float32 y~%float32 z~%float32 errno~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BodyStatus>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'object_id))
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BodyStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'BodyStatus
    (cl:cons ':object_id (object_id msg))
    (cl:cons ':picked_up (picked_up msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':errno (errno msg))
))
