; Auto-generated. Do not edit!


(cl:in-package dynamixel_hardware_interface-srv)


;//! \htmlinclude SetTorqueLimit-request.msg.html

(cl:defclass <SetTorqueLimit-request> (roslisp-msg-protocol:ros-message)
  ((torque_limit
    :reader torque_limit
    :initarg :torque_limit
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetTorqueLimit-request (<SetTorqueLimit-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTorqueLimit-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTorqueLimit-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_hardware_interface-srv:<SetTorqueLimit-request> is deprecated: use dynamixel_hardware_interface-srv:SetTorqueLimit-request instead.")))

(cl:ensure-generic-function 'torque_limit-val :lambda-list '(m))
(cl:defmethod torque_limit-val ((m <SetTorqueLimit-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_hardware_interface-srv:torque_limit-val is deprecated.  Use dynamixel_hardware_interface-srv:torque_limit instead.")
  (torque_limit m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTorqueLimit-request>) ostream)
  "Serializes a message object of type '<SetTorqueLimit-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'torque_limit))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTorqueLimit-request>) istream)
  "Deserializes a message object of type '<SetTorqueLimit-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque_limit) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTorqueLimit-request>)))
  "Returns string type for a service object of type '<SetTorqueLimit-request>"
  "dynamixel_hardware_interface/SetTorqueLimitRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTorqueLimit-request)))
  "Returns string type for a service object of type 'SetTorqueLimit-request"
  "dynamixel_hardware_interface/SetTorqueLimitRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTorqueLimit-request>)))
  "Returns md5sum for a message object of type '<SetTorqueLimit-request>"
  "7ac67170532bb79d95db2a425915bbd6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTorqueLimit-request)))
  "Returns md5sum for a message object of type 'SetTorqueLimit-request"
  "7ac67170532bb79d95db2a425915bbd6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTorqueLimit-request>)))
  "Returns full string definition for message of type '<SetTorqueLimit-request>"
  (cl:format cl:nil "~%~%float64 torque_limit~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTorqueLimit-request)))
  "Returns full string definition for message of type 'SetTorqueLimit-request"
  (cl:format cl:nil "~%~%float64 torque_limit~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTorqueLimit-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTorqueLimit-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTorqueLimit-request
    (cl:cons ':torque_limit (torque_limit msg))
))
;//! \htmlinclude SetTorqueLimit-response.msg.html

(cl:defclass <SetTorqueLimit-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetTorqueLimit-response (<SetTorqueLimit-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTorqueLimit-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTorqueLimit-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_hardware_interface-srv:<SetTorqueLimit-response> is deprecated: use dynamixel_hardware_interface-srv:SetTorqueLimit-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTorqueLimit-response>) ostream)
  "Serializes a message object of type '<SetTorqueLimit-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTorqueLimit-response>) istream)
  "Deserializes a message object of type '<SetTorqueLimit-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTorqueLimit-response>)))
  "Returns string type for a service object of type '<SetTorqueLimit-response>"
  "dynamixel_hardware_interface/SetTorqueLimitResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTorqueLimit-response)))
  "Returns string type for a service object of type 'SetTorqueLimit-response"
  "dynamixel_hardware_interface/SetTorqueLimitResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTorqueLimit-response>)))
  "Returns md5sum for a message object of type '<SetTorqueLimit-response>"
  "7ac67170532bb79d95db2a425915bbd6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTorqueLimit-response)))
  "Returns md5sum for a message object of type 'SetTorqueLimit-response"
  "7ac67170532bb79d95db2a425915bbd6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTorqueLimit-response>)))
  "Returns full string definition for message of type '<SetTorqueLimit-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTorqueLimit-response)))
  "Returns full string definition for message of type 'SetTorqueLimit-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTorqueLimit-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTorqueLimit-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTorqueLimit-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetTorqueLimit)))
  'SetTorqueLimit-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetTorqueLimit)))
  'SetTorqueLimit-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTorqueLimit)))
  "Returns string type for a service object of type '<SetTorqueLimit>"
  "dynamixel_hardware_interface/SetTorqueLimit")