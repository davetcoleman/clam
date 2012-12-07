; Auto-generated. Do not edit!


(cl:in-package dynamixel_hardware_interface-srv)


;//! \htmlinclude TorqueEnable-request.msg.html

(cl:defclass <TorqueEnable-request> (roslisp-msg-protocol:ros-message)
  ((torque_enable
    :reader torque_enable
    :initarg :torque_enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass TorqueEnable-request (<TorqueEnable-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TorqueEnable-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TorqueEnable-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_hardware_interface-srv:<TorqueEnable-request> is deprecated: use dynamixel_hardware_interface-srv:TorqueEnable-request instead.")))

(cl:ensure-generic-function 'torque_enable-val :lambda-list '(m))
(cl:defmethod torque_enable-val ((m <TorqueEnable-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_hardware_interface-srv:torque_enable-val is deprecated.  Use dynamixel_hardware_interface-srv:torque_enable instead.")
  (torque_enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TorqueEnable-request>) ostream)
  "Serializes a message object of type '<TorqueEnable-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'torque_enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TorqueEnable-request>) istream)
  "Deserializes a message object of type '<TorqueEnable-request>"
    (cl:setf (cl:slot-value msg 'torque_enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TorqueEnable-request>)))
  "Returns string type for a service object of type '<TorqueEnable-request>"
  "dynamixel_hardware_interface/TorqueEnableRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TorqueEnable-request)))
  "Returns string type for a service object of type 'TorqueEnable-request"
  "dynamixel_hardware_interface/TorqueEnableRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TorqueEnable-request>)))
  "Returns md5sum for a message object of type '<TorqueEnable-request>"
  "e44dc96db32bd58b5a896c2c5bf316d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TorqueEnable-request)))
  "Returns md5sum for a message object of type 'TorqueEnable-request"
  "e44dc96db32bd58b5a896c2c5bf316d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TorqueEnable-request>)))
  "Returns full string definition for message of type '<TorqueEnable-request>"
  (cl:format cl:nil "bool torque_enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TorqueEnable-request)))
  "Returns full string definition for message of type 'TorqueEnable-request"
  (cl:format cl:nil "bool torque_enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TorqueEnable-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TorqueEnable-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TorqueEnable-request
    (cl:cons ':torque_enable (torque_enable msg))
))
;//! \htmlinclude TorqueEnable-response.msg.html

(cl:defclass <TorqueEnable-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TorqueEnable-response (<TorqueEnable-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TorqueEnable-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TorqueEnable-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_hardware_interface-srv:<TorqueEnable-response> is deprecated: use dynamixel_hardware_interface-srv:TorqueEnable-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TorqueEnable-response>) ostream)
  "Serializes a message object of type '<TorqueEnable-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TorqueEnable-response>) istream)
  "Deserializes a message object of type '<TorqueEnable-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TorqueEnable-response>)))
  "Returns string type for a service object of type '<TorqueEnable-response>"
  "dynamixel_hardware_interface/TorqueEnableResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TorqueEnable-response)))
  "Returns string type for a service object of type 'TorqueEnable-response"
  "dynamixel_hardware_interface/TorqueEnableResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TorqueEnable-response>)))
  "Returns md5sum for a message object of type '<TorqueEnable-response>"
  "e44dc96db32bd58b5a896c2c5bf316d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TorqueEnable-response)))
  "Returns md5sum for a message object of type 'TorqueEnable-response"
  "e44dc96db32bd58b5a896c2c5bf316d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TorqueEnable-response>)))
  "Returns full string definition for message of type '<TorqueEnable-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TorqueEnable-response)))
  "Returns full string definition for message of type 'TorqueEnable-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TorqueEnable-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TorqueEnable-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TorqueEnable-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TorqueEnable)))
  'TorqueEnable-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TorqueEnable)))
  'TorqueEnable-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TorqueEnable)))
  "Returns string type for a service object of type '<TorqueEnable>"
  "dynamixel_hardware_interface/TorqueEnable")