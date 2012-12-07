; Auto-generated. Do not edit!


(cl:in-package dynamixel_hardware_interface-srv)


;//! \htmlinclude SetComplianceMargin-request.msg.html

(cl:defclass <SetComplianceMargin-request> (roslisp-msg-protocol:ros-message)
  ((margin
    :reader margin
    :initarg :margin
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetComplianceMargin-request (<SetComplianceMargin-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetComplianceMargin-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetComplianceMargin-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_hardware_interface-srv:<SetComplianceMargin-request> is deprecated: use dynamixel_hardware_interface-srv:SetComplianceMargin-request instead.")))

(cl:ensure-generic-function 'margin-val :lambda-list '(m))
(cl:defmethod margin-val ((m <SetComplianceMargin-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_hardware_interface-srv:margin-val is deprecated.  Use dynamixel_hardware_interface-srv:margin instead.")
  (margin m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetComplianceMargin-request>) ostream)
  "Serializes a message object of type '<SetComplianceMargin-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'margin)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetComplianceMargin-request>) istream)
  "Deserializes a message object of type '<SetComplianceMargin-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'margin)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetComplianceMargin-request>)))
  "Returns string type for a service object of type '<SetComplianceMargin-request>"
  "dynamixel_hardware_interface/SetComplianceMarginRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetComplianceMargin-request)))
  "Returns string type for a service object of type 'SetComplianceMargin-request"
  "dynamixel_hardware_interface/SetComplianceMarginRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetComplianceMargin-request>)))
  "Returns md5sum for a message object of type '<SetComplianceMargin-request>"
  "daacbf1c0642fe923f2dfb9217a97b81")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetComplianceMargin-request)))
  "Returns md5sum for a message object of type 'SetComplianceMargin-request"
  "daacbf1c0642fe923f2dfb9217a97b81")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetComplianceMargin-request>)))
  "Returns full string definition for message of type '<SetComplianceMargin-request>"
  (cl:format cl:nil "~%uint8 margin~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetComplianceMargin-request)))
  "Returns full string definition for message of type 'SetComplianceMargin-request"
  (cl:format cl:nil "~%uint8 margin~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetComplianceMargin-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetComplianceMargin-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetComplianceMargin-request
    (cl:cons ':margin (margin msg))
))
;//! \htmlinclude SetComplianceMargin-response.msg.html

(cl:defclass <SetComplianceMargin-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetComplianceMargin-response (<SetComplianceMargin-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetComplianceMargin-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetComplianceMargin-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_hardware_interface-srv:<SetComplianceMargin-response> is deprecated: use dynamixel_hardware_interface-srv:SetComplianceMargin-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetComplianceMargin-response>) ostream)
  "Serializes a message object of type '<SetComplianceMargin-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetComplianceMargin-response>) istream)
  "Deserializes a message object of type '<SetComplianceMargin-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetComplianceMargin-response>)))
  "Returns string type for a service object of type '<SetComplianceMargin-response>"
  "dynamixel_hardware_interface/SetComplianceMarginResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetComplianceMargin-response)))
  "Returns string type for a service object of type 'SetComplianceMargin-response"
  "dynamixel_hardware_interface/SetComplianceMarginResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetComplianceMargin-response>)))
  "Returns md5sum for a message object of type '<SetComplianceMargin-response>"
  "daacbf1c0642fe923f2dfb9217a97b81")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetComplianceMargin-response)))
  "Returns md5sum for a message object of type 'SetComplianceMargin-response"
  "daacbf1c0642fe923f2dfb9217a97b81")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetComplianceMargin-response>)))
  "Returns full string definition for message of type '<SetComplianceMargin-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetComplianceMargin-response)))
  "Returns full string definition for message of type 'SetComplianceMargin-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetComplianceMargin-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetComplianceMargin-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetComplianceMargin-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetComplianceMargin)))
  'SetComplianceMargin-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetComplianceMargin)))
  'SetComplianceMargin-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetComplianceMargin)))
  "Returns string type for a service object of type '<SetComplianceMargin>"
  "dynamixel_hardware_interface/SetComplianceMargin")