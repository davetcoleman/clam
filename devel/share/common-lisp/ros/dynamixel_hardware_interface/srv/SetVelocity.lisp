; Auto-generated. Do not edit!


(cl:in-package dynamixel_hardware_interface-srv)


;//! \htmlinclude SetVelocity-request.msg.html

(cl:defclass <SetVelocity-request> (roslisp-msg-protocol:ros-message)
  ((velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetVelocity-request (<SetVelocity-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVelocity-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVelocity-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_hardware_interface-srv:<SetVelocity-request> is deprecated: use dynamixel_hardware_interface-srv:SetVelocity-request instead.")))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <SetVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_hardware_interface-srv:velocity-val is deprecated.  Use dynamixel_hardware_interface-srv:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVelocity-request>) ostream)
  "Serializes a message object of type '<SetVelocity-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVelocity-request>) istream)
  "Deserializes a message object of type '<SetVelocity-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVelocity-request>)))
  "Returns string type for a service object of type '<SetVelocity-request>"
  "dynamixel_hardware_interface/SetVelocityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVelocity-request)))
  "Returns string type for a service object of type 'SetVelocity-request"
  "dynamixel_hardware_interface/SetVelocityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVelocity-request>)))
  "Returns md5sum for a message object of type '<SetVelocity-request>"
  "6d3b2ef8452c2fe21adb709618d2518f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVelocity-request)))
  "Returns md5sum for a message object of type 'SetVelocity-request"
  "6d3b2ef8452c2fe21adb709618d2518f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVelocity-request>)))
  "Returns full string definition for message of type '<SetVelocity-request>"
  (cl:format cl:nil "float64 velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVelocity-request)))
  "Returns full string definition for message of type 'SetVelocity-request"
  (cl:format cl:nil "float64 velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVelocity-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVelocity-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVelocity-request
    (cl:cons ':velocity (velocity msg))
))
;//! \htmlinclude SetVelocity-response.msg.html

(cl:defclass <SetVelocity-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetVelocity-response (<SetVelocity-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVelocity-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVelocity-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_hardware_interface-srv:<SetVelocity-response> is deprecated: use dynamixel_hardware_interface-srv:SetVelocity-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVelocity-response>) ostream)
  "Serializes a message object of type '<SetVelocity-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVelocity-response>) istream)
  "Deserializes a message object of type '<SetVelocity-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVelocity-response>)))
  "Returns string type for a service object of type '<SetVelocity-response>"
  "dynamixel_hardware_interface/SetVelocityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVelocity-response)))
  "Returns string type for a service object of type 'SetVelocity-response"
  "dynamixel_hardware_interface/SetVelocityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVelocity-response>)))
  "Returns md5sum for a message object of type '<SetVelocity-response>"
  "6d3b2ef8452c2fe21adb709618d2518f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVelocity-response)))
  "Returns md5sum for a message object of type 'SetVelocity-response"
  "6d3b2ef8452c2fe21adb709618d2518f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVelocity-response>)))
  "Returns full string definition for message of type '<SetVelocity-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVelocity-response)))
  "Returns full string definition for message of type 'SetVelocity-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVelocity-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVelocity-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVelocity-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetVelocity)))
  'SetVelocity-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetVelocity)))
  'SetVelocity-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVelocity)))
  "Returns string type for a service object of type '<SetVelocity>"
  "dynamixel_hardware_interface/SetVelocity")