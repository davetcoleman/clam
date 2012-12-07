
(cl:in-package :asdf)

(defsystem "dynamixel_hardware_interface-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JointState" :depends-on ("_package_JointState"))
    (:file "_package_JointState" :depends-on ("_package"))
    (:file "MotorStateList" :depends-on ("_package_MotorStateList"))
    (:file "_package_MotorStateList" :depends-on ("_package"))
    (:file "MotorState" :depends-on ("_package_MotorState"))
    (:file "_package_MotorState" :depends-on ("_package"))
  ))