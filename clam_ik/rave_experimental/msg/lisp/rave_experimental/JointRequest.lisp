; Auto-generated. Do not edit!


(in-package rave_experimental-msg)


;//! \htmlinclude JointRequest.msg.html

(defclass <JointRequest> (ros-message)
  ()
)
(defmethod serialize ((msg <JointRequest>) ostream)
  "Serializes a message object of type '<JointRequest>"
)
(defmethod deserialize ((msg <JointRequest>) istream)
  "Deserializes a message object of type '<JointRequest>"
  msg
)
(defmethod ros-datatype ((msg (eql '<JointRequest>)))
  "Returns string type for a message object of type '<JointRequest>"
  "rave_experimental/JointRequest")
(defmethod md5sum ((type (eql '<JointRequest>)))
  "Returns md5sum for a message object of type '<JointRequest>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<JointRequest>)))
  "Returns full string definition for message of type '<JointRequest>"
  (format nil "~%~%"))
(defmethod serialization-length ((msg <JointRequest>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <JointRequest>))
  "Converts a ROS message object to a list"
  (list '<JointRequest>
))
