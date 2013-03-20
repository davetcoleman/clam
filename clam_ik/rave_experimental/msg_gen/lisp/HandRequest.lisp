; Auto-generated. Do not edit!


(cl:in-package rave_experimental-msg)


;//! \htmlinclude HandRequest.msg.html

(cl:defclass <HandRequest> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass HandRequest (<HandRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rave_experimental-msg:<HandRequest> is deprecated: use rave_experimental-msg:HandRequest instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandRequest>) ostream)
  "Serializes a message object of type '<HandRequest>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandRequest>) istream)
  "Deserializes a message object of type '<HandRequest>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandRequest>)))
  "Returns string type for a message object of type '<HandRequest>"
  "rave_experimental/HandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandRequest)))
  "Returns string type for a message object of type 'HandRequest"
  "rave_experimental/HandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandRequest>)))
  "Returns md5sum for a message object of type '<HandRequest>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandRequest)))
  "Returns md5sum for a message object of type 'HandRequest"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandRequest>)))
  "Returns full string definition for message of type '<HandRequest>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandRequest)))
  "Returns full string definition for message of type 'HandRequest"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandRequest>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'HandRequest
))
