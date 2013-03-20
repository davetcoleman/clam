; Auto-generated. Do not edit!


(in-package rave_experimental-msg)


;//! \htmlinclude EnviromentUpdate.msg.html

(defclass <EnviromentUpdate> (ros-message)
  ((object_id
    :reader object_id-val
    :initarg :object_id
    :type string
    :initform "")
   (xml_file
    :reader xml_file-val
    :initarg :xml_file
    :type string
    :initform "")
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
   (roll
    :reader roll-val
    :initarg :roll
    :type float
    :initform 0.0)
   (pitch
    :reader pitch-val
    :initarg :pitch
    :type float
    :initform 0.0)
   (yaw
    :reader yaw-val
    :initarg :yaw
    :type float
    :initform 0.0)
   (camera_relative
    :reader camera_relative-val
    :initarg :camera_relative
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <EnviromentUpdate>) ostream)
  "Serializes a message object of type '<EnviromentUpdate>"
  (let ((__ros_str_len (length (slot-value msg 'object_id))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'object_id))
  (let ((__ros_str_len (length (slot-value msg 'xml_file))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'xml_file))
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
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'roll))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'pitch))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'yaw))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'camera_relative) 1 0)) ostream)
)
(defmethod deserialize ((msg <EnviromentUpdate>) istream)
  "Deserializes a message object of type '<EnviromentUpdate>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'object_id) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'object_id) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'xml_file) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'xml_file) __ros_str_idx) (code-char (read-byte istream)))))
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
    (setf (slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  (setf (slot-value msg 'camera_relative) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<EnviromentUpdate>)))
  "Returns string type for a message object of type '<EnviromentUpdate>"
  "rave_experimental/EnviromentUpdate")
(defmethod md5sum ((type (eql '<EnviromentUpdate>)))
  "Returns md5sum for a message object of type '<EnviromentUpdate>"
  "f4507a3b88a2f180859a169173f77e45")
(defmethod message-definition ((type (eql '<EnviromentUpdate>)))
  "Returns full string definition for message of type '<EnviromentUpdate>"
  (format nil "string object_id~%string xml_file~%float32 x~%float32 y~%float32 z~%float32 roll~%float32 pitch~%float32 yaw~%bool camera_relative~%~%~%"))
(defmethod serialization-length ((msg <EnviromentUpdate>))
  (+ 0
     4 (length (slot-value msg 'object_id))
     4 (length (slot-value msg 'xml_file))
     4
     4
     4
     4
     4
     4
     1
))
(defmethod ros-message-to-list ((msg <EnviromentUpdate>))
  "Converts a ROS message object to a list"
  (list '<EnviromentUpdate>
    (cons ':object_id (object_id-val msg))
    (cons ':xml_file (xml_file-val msg))
    (cons ':x (x-val msg))
    (cons ':y (y-val msg))
    (cons ':z (z-val msg))
    (cons ':roll (roll-val msg))
    (cons ':pitch (pitch-val msg))
    (cons ':yaw (yaw-val msg))
    (cons ':camera_relative (camera_relative-val msg))
))
