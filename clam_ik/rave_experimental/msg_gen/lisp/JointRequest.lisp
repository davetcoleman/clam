; Auto-generated. Do not edit!


(cl:in-package rave_experimental-msg)


;//! \htmlinclude JointRequest.msg.html

(cl:defclass <JointRequest> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass JointRequest (<JointRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rave_experimental-msg:<JointRequest> is deprecated: use rave_experimental-msg:JointRequest instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointRequest>) ostream)
  "Serializes a message object of type '<JointRequest>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointRequest>) istream)
  "Deserializes a message object of type '<JointRequest>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointRequest>)))
  "Returns string type for a message object of type '<JointRequest>"
  "rave_experimental/JointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointRequest)))
  "Returns string type for a message object of type 'JointRequest"
  "rave_experimental/JointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointRequest>)))
  "Returns md5sum for a message object of type '<JointRequest>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointRequest)))
  "Returns md5sum for a message object of type 'JointRequest"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointRequest>)))
  "Returns full string definition for message of type '<JointRequest>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointRequest)))
  "Returns full string definition for message of type 'JointRequest"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointRequest>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'JointRequest
))
