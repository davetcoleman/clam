; Auto-generated. Do not edit!


(in-package rave_experimental-msg)


;//! \htmlinclude CameraPositionRequest.msg.html

(defclass <CameraPositionRequest> (ros-message)
  ()
)
(defmethod serialize ((msg <CameraPositionRequest>) ostream)
  "Serializes a message object of type '<CameraPositionRequest>"
)
(defmethod deserialize ((msg <CameraPositionRequest>) istream)
  "Deserializes a message object of type '<CameraPositionRequest>"
  msg
)
(defmethod ros-datatype ((msg (eql '<CameraPositionRequest>)))
  "Returns string type for a message object of type '<CameraPositionRequest>"
  "rave_experimental/CameraPositionRequest")
(defmethod md5sum ((type (eql '<CameraPositionRequest>)))
  "Returns md5sum for a message object of type '<CameraPositionRequest>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<CameraPositionRequest>)))
  "Returns full string definition for message of type '<CameraPositionRequest>"
  (format nil "~%~%"))
(defmethod serialization-length ((msg <CameraPositionRequest>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <CameraPositionRequest>))
  "Converts a ROS message object to a list"
  (list '<CameraPositionRequest>
))
