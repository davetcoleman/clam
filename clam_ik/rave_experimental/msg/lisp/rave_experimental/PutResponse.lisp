; Auto-generated. Do not edit!


(in-package rave_experimental-msg)


;//! \htmlinclude PutResponse.msg.html

(defclass <PutResponse> (ros-message)
  ((object_id
    :reader object_id-val
    :initarg :object_id
    :type string
    :initform "")
   (errno
    :reader errno-val
    :initarg :errno
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <PutResponse>) ostream)
  "Serializes a message object of type '<PutResponse>"
  (let ((__ros_str_len (length (slot-value msg 'object_id))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'object_id))
    (write-byte (ldb (byte 8 0) (slot-value msg 'errno)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'errno)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'errno)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'errno)) ostream)
)
(defmethod deserialize ((msg <PutResponse>) istream)
  "Deserializes a message object of type '<PutResponse>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'object_id) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'object_id) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'errno)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'errno)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'errno)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'errno)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<PutResponse>)))
  "Returns string type for a message object of type '<PutResponse>"
  "rave_experimental/PutResponse")
(defmethod md5sum ((type (eql '<PutResponse>)))
  "Returns md5sum for a message object of type '<PutResponse>"
  "9e7f17b0f2355c303da59bb9ccc1d435")
(defmethod message-definition ((type (eql '<PutResponse>)))
  "Returns full string definition for message of type '<PutResponse>"
  (format nil "string object_id~%uint32 errno~%~%~%"))
(defmethod serialization-length ((msg <PutResponse>))
  (+ 0
     4 (length (slot-value msg 'object_id))
     4
))
(defmethod ros-message-to-list ((msg <PutResponse>))
  "Converts a ROS message object to a list"
  (list '<PutResponse>
    (cons ':object_id (object_id-val msg))
    (cons ':errno (errno-val msg))
))
