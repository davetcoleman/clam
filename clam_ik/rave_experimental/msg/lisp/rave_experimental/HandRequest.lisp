; Auto-generated. Do not edit!


(in-package rave_experimental-msg)


;//! \htmlinclude HandRequest.msg.html

(defclass <HandRequest> (ros-message)
  ()
)
(defmethod serialize ((msg <HandRequest>) ostream)
  "Serializes a message object of type '<HandRequest>"
)
(defmethod deserialize ((msg <HandRequest>) istream)
  "Deserializes a message object of type '<HandRequest>"
  msg
)
(defmethod ros-datatype ((msg (eql '<HandRequest>)))
  "Returns string type for a message object of type '<HandRequest>"
  "rave_experimental/HandRequest")
(defmethod md5sum ((type (eql '<HandRequest>)))
  "Returns md5sum for a message object of type '<HandRequest>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<HandRequest>)))
  "Returns full string definition for message of type '<HandRequest>"
  (format nil "~%~%"))
(defmethod serialization-length ((msg <HandRequest>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <HandRequest>))
  "Converts a ROS message object to a list"
  (list '<HandRequest>
))
