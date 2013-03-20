; Auto-generated. Do not edit!


(cl:in-package rave_experimental-msg)


;//! \htmlinclude EnviromentUpdate.msg.html

(cl:defclass <EnviromentUpdate> (roslisp-msg-protocol:ros-message)
  ((object_id
    :reader object_id
    :initarg :object_id
    :type cl:string
    :initform "")
   (xml_file
    :reader xml_file
    :initarg :xml_file
    :type cl:string
    :initform "")
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
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (camera_relative
    :reader camera_relative
    :initarg :camera_relative
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass EnviromentUpdate (<EnviromentUpdate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnviromentUpdate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnviromentUpdate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rave_experimental-msg:<EnviromentUpdate> is deprecated: use rave_experimental-msg:EnviromentUpdate instead.")))

(cl:ensure-generic-function 'object_id-val :lambda-list '(m))
(cl:defmethod object_id-val ((m <EnviromentUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:object_id-val is deprecated.  Use rave_experimental-msg:object_id instead.")
  (object_id m))

(cl:ensure-generic-function 'xml_file-val :lambda-list '(m))
(cl:defmethod xml_file-val ((m <EnviromentUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:xml_file-val is deprecated.  Use rave_experimental-msg:xml_file instead.")
  (xml_file m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <EnviromentUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:x-val is deprecated.  Use rave_experimental-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <EnviromentUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:y-val is deprecated.  Use rave_experimental-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <EnviromentUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:z-val is deprecated.  Use rave_experimental-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <EnviromentUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:roll-val is deprecated.  Use rave_experimental-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <EnviromentUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:pitch-val is deprecated.  Use rave_experimental-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <EnviromentUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:yaw-val is deprecated.  Use rave_experimental-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'camera_relative-val :lambda-list '(m))
(cl:defmethod camera_relative-val ((m <EnviromentUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:camera_relative-val is deprecated.  Use rave_experimental-msg:camera_relative instead.")
  (camera_relative m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnviromentUpdate>) ostream)
  "Serializes a message object of type '<EnviromentUpdate>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object_id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'xml_file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'xml_file))
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'camera_relative) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnviromentUpdate>) istream)
  "Deserializes a message object of type '<EnviromentUpdate>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'xml_file) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'xml_file) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'camera_relative) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnviromentUpdate>)))
  "Returns string type for a message object of type '<EnviromentUpdate>"
  "rave_experimental/EnviromentUpdate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnviromentUpdate)))
  "Returns string type for a message object of type 'EnviromentUpdate"
  "rave_experimental/EnviromentUpdate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnviromentUpdate>)))
  "Returns md5sum for a message object of type '<EnviromentUpdate>"
  "f4507a3b88a2f180859a169173f77e45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnviromentUpdate)))
  "Returns md5sum for a message object of type 'EnviromentUpdate"
  "f4507a3b88a2f180859a169173f77e45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnviromentUpdate>)))
  "Returns full string definition for message of type '<EnviromentUpdate>"
  (cl:format cl:nil "string object_id~%string xml_file~%float32 x~%float32 y~%float32 z~%float32 roll~%float32 pitch~%float32 yaw~%bool camera_relative~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnviromentUpdate)))
  "Returns full string definition for message of type 'EnviromentUpdate"
  (cl:format cl:nil "string object_id~%string xml_file~%float32 x~%float32 y~%float32 z~%float32 roll~%float32 pitch~%float32 yaw~%bool camera_relative~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnviromentUpdate>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'object_id))
     4 (cl:length (cl:slot-value msg 'xml_file))
     4
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnviromentUpdate>))
  "Converts a ROS message object to a list"
  (cl:list 'EnviromentUpdate
    (cl:cons ':object_id (object_id msg))
    (cl:cons ':xml_file (xml_file msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':camera_relative (camera_relative msg))
))
