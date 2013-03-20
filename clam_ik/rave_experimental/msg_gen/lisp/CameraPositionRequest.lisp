; Auto-generated. Do not edit!


(cl:in-package rave_experimental-msg)


;//! \htmlinclude CameraPositionRequest.msg.html

(cl:defclass <CameraPositionRequest> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CameraPositionRequest (<CameraPositionRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraPositionRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraPositionRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rave_experimental-msg:<CameraPositionRequest> is deprecated: use rave_experimental-msg:CameraPositionRequest instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraPositionRequest>) ostream)
  "Serializes a message object of type '<CameraPositionRequest>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraPositionRequest>) istream)
  "Deserializes a message object of type '<CameraPositionRequest>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraPositionRequest>)))
  "Returns string type for a message object of type '<CameraPositionRequest>"
  "rave_experimental/CameraPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraPositionRequest)))
  "Returns string type for a message object of type 'CameraPositionRequest"
  "rave_experimental/CameraPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraPositionRequest>)))
  "Returns md5sum for a message object of type '<CameraPositionRequest>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraPositionRequest)))
  "Returns md5sum for a message object of type 'CameraPositionRequest"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraPositionRequest>)))
  "Returns full string definition for message of type '<CameraPositionRequest>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraPositionRequest)))
  "Returns full string definition for message of type 'CameraPositionRequest"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraPositionRequest>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraPositionRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraPositionRequest
))
