; Auto-generated. Do not edit!


(in-package rave_experimental-msg)


;//! \htmlinclude GrabCommand.msg.html

(defclass <GrabCommand> (ros-message)
  ((object_id
    :reader object_id-val
    :initarg :object_id
    :type string
    :initform "")
   (use_visual_feedback
    :reader use_visual_feedback-val
    :initarg :use_visual_feedback
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <GrabCommand>) ostream)
  "Serializes a message object of type '<GrabCommand>"
  (let ((__ros_str_len (length (slot-value msg 'object_id))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'object_id))
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'use_visual_feedback) 1 0)) ostream)
)
(defmethod deserialize ((msg <GrabCommand>) istream)
  "Deserializes a message object of type '<GrabCommand>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'object_id) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'object_id) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (slot-value msg 'use_visual_feedback) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<GrabCommand>)))
  "Returns string type for a message object of type '<GrabCommand>"
  "rave_experimental/GrabCommand")
(defmethod md5sum ((type (eql '<GrabCommand>)))
  "Returns md5sum for a message object of type '<GrabCommand>"
  "5da422b82edbbff9f2e278bcc00f2eb3")
(defmethod message-definition ((type (eql '<GrabCommand>)))
  "Returns full string definition for message of type '<GrabCommand>"
  (format nil "string object_id~%bool use_visual_feedback~%~%~%"))
(defmethod serialization-length ((msg <GrabCommand>))
  (+ 0
     4 (length (slot-value msg 'object_id))
     1
))
(defmethod ros-message-to-list ((msg <GrabCommand>))
  "Converts a ROS message object to a list"
  (list '<GrabCommand>
    (cons ':object_id (object_id-val msg))
    (cons ':use_visual_feedback (use_visual_feedback-val msg))
))
