; Auto-generated. Do not edit!


(in-package rave_experimental-msg)


;//! \htmlinclude Update.msg.html

(defclass <Update> (ros-message)
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
   (grab
    :reader grab-val
    :initarg :grab
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <Update>) ostream)
  "Serializes a message object of type '<Update>"
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
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'grab) 1 0)) ostream)
)
(defmethod deserialize ((msg <Update>) istream)
  "Deserializes a message object of type '<Update>"
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
  (setf (slot-value msg 'grab) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Update>)))
  "Returns string type for a message object of type '<Update>"
  "rave_experimental/Update")
(defmethod md5sum ((type (eql '<Update>)))
  "Returns md5sum for a message object of type '<Update>"
  "2cc603712d0fb9a75a2efdbd218b3c80")
(defmethod message-definition ((type (eql '<Update>)))
  "Returns full string definition for message of type '<Update>"
  (format nil "string object_id~%string xml_file~%float32 x~%float32 y~%float32 z~%bool grab~%~%~%"))
(defmethod serialization-length ((msg <Update>))
  (+ 0
     4 (length (slot-value msg 'object_id))
     4 (length (slot-value msg 'xml_file))
     4
     4
     4
     1
))
(defmethod ros-message-to-list ((msg <Update>))
  "Converts a ROS message object to a list"
  (list '<Update>
    (cons ':object_id (object_id-val msg))
    (cons ':xml_file (xml_file-val msg))
    (cons ':x (x-val msg))
    (cons ':y (y-val msg))
    (cons ':z (z-val msg))
    (cons ':grab (grab-val msg))
))
