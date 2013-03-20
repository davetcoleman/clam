; Auto-generated. Do not edit!


(cl:in-package rave_experimental-msg)


;//! \htmlinclude GrabCommand.msg.html

(cl:defclass <GrabCommand> (roslisp-msg-protocol:ros-message)
  ((object_id
    :reader object_id
    :initarg :object_id
    :type cl:string
    :initform "")
   (use_visual_feedback
    :reader use_visual_feedback
    :initarg :use_visual_feedback
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GrabCommand (<GrabCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GrabCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GrabCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rave_experimental-msg:<GrabCommand> is deprecated: use rave_experimental-msg:GrabCommand instead.")))

(cl:ensure-generic-function 'object_id-val :lambda-list '(m))
(cl:defmethod object_id-val ((m <GrabCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:object_id-val is deprecated.  Use rave_experimental-msg:object_id instead.")
  (object_id m))

(cl:ensure-generic-function 'use_visual_feedback-val :lambda-list '(m))
(cl:defmethod use_visual_feedback-val ((m <GrabCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rave_experimental-msg:use_visual_feedback-val is deprecated.  Use rave_experimental-msg:use_visual_feedback instead.")
  (use_visual_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GrabCommand>) ostream)
  "Serializes a message object of type '<GrabCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object_id))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_visual_feedback) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GrabCommand>) istream)
  "Deserializes a message object of type '<GrabCommand>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'use_visual_feedback) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GrabCommand>)))
  "Returns string type for a message object of type '<GrabCommand>"
  "rave_experimental/GrabCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GrabCommand)))
  "Returns string type for a message object of type 'GrabCommand"
  "rave_experimental/GrabCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GrabCommand>)))
  "Returns md5sum for a message object of type '<GrabCommand>"
  "5da422b82edbbff9f2e278bcc00f2eb3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GrabCommand)))
  "Returns md5sum for a message object of type 'GrabCommand"
  "5da422b82edbbff9f2e278bcc00f2eb3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GrabCommand>)))
  "Returns full string definition for message of type '<GrabCommand>"
  (cl:format cl:nil "string object_id~%bool use_visual_feedback~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GrabCommand)))
  "Returns full string definition for message of type 'GrabCommand"
  (cl:format cl:nil "string object_id~%bool use_visual_feedback~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GrabCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'object_id))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GrabCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'GrabCommand
    (cl:cons ':object_id (object_id msg))
    (cl:cons ':use_visual_feedback (use_visual_feedback msg))
))
