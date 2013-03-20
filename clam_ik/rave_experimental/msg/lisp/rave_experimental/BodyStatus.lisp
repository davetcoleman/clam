; Auto-generated. Do not edit!


(in-package rave_experimental-msg)


;//! \htmlinclude BodyStatus.msg.html

(defclass <BodyStatus> (ros-message)
  ((object_id
    :reader object_id-val
    :initarg :object_id
    :type string
    :initform "")
   (picked_up
    :reader picked_up-val
    :initarg :picked_up
    :type boolean
    :initform nil)
   (x
    :reader x-val
    :initarg :x
    :type float
    :initform 0.0)
   (y
    :reader y-val
    :initarg :y
    :type float
    :initform 0.0)
   (z
    :reader z-val
    :initarg :z
    :type float
    :initform 0.0)
   (errno
    :reader errno-val
    :initarg :errno
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <BodyStatus>) ostream)
  "Serializes a message object of type '<BodyStatus>"
  (let ((__ros_str_len (length (slot-value msg 'object_id))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'object_id))
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'picked_up) 1 0)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'x))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'y))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'z))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'errno))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <BodyStatus>) istream)
  "Deserializes a message object of type '<BodyStatus>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'object_id) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'object_id) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (slot-value msg 'picked_up) (not (zerop (read-byte istream))))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'errno) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<BodyStatus>)))
  "Returns string type for a message object of type '<BodyStatus>"
  "rave_experimental/BodyStatus")
(defmethod md5sum ((type (eql '<BodyStatus>)))
  "Returns md5sum for a message object of type '<BodyStatus>"
  "9d67177aea4c214587c878f7ce226d1f")
(defmethod message-definition ((type (eql '<BodyStatus>)))
  "Returns full string definition for message of type '<BodyStatus>"
  (format nil "string object_id~%bool picked_up~%float32 x~%float32 y~%float32 z~%float32 errno~%~%~%"))
(defmethod serialization-length ((msg <BodyStatus>))
  (+ 0
     4 (length (slot-value msg 'object_id))
     1
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <BodyStatus>))
  "Converts a ROS message object to a list"
  (list '<BodyStatus>
    (cons ':object_id (object_id-val msg))
    (cons ':picked_up (picked_up-val msg))
    (cons ':x (x-val msg))
    (cons ':y (y-val msg))
    (cons ':z (z-val msg))
    (cons ':errno (errno-val msg))
))
