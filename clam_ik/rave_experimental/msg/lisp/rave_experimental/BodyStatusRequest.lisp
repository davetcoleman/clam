; Auto-generated. Do not edit!


(in-package rave_experimental-msg)


;//! \htmlinclude BodyStatusRequest.msg.html

(defclass <BodyStatusRequest> (ros-message)
  ((object_id
    :reader object_id-val
    :initarg :object_id
    :type string
    :initform ""))
)
(defmethod serialize ((msg <BodyStatusRequest>) ostream)
  "Serializes a message object of type '<BodyStatusRequest>"
  (let ((__ros_str_len (length (slot-value msg 'object_id))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'object_id))
)
(defmethod deserialize ((msg <BodyStatusRequest>) istream)
  "Deserializes a message object of type '<BodyStatusRequest>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'object_id) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'object_id) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<BodyStatusRequest>)))
  "Returns string type for a message object of type '<BodyStatusRequest>"
  "rave_experimental/BodyStatusRequest")
(defmethod md5sum ((type (eql '<BodyStatusRequest>)))
  "Returns md5sum for a message object of type '<BodyStatusRequest>"
  "d552cd960e0a22b19c9db18033cea0eb")
(defmethod message-definition ((type (eql '<BodyStatusRequest>)))
  "Returns full string definition for message of type '<BodyStatusRequest>"
  (format nil "string object_id~%~%~%"))
(defmethod serialization-length ((msg <BodyStatusRequest>))
  (+ 0
     4 (length (slot-value msg 'object_id))
))
(defmethod ros-message-to-list ((msg <BodyStatusRequest>))
  "Converts a ROS message object to a list"
  (list '<BodyStatusRequest>
    (cons ':object_id (object_id-val msg))
))
