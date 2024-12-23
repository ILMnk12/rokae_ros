
(cl:in-package :asdf)

(defsystem "rokae_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "ExternalForce" :depends-on ("_package_ExternalForce"))
    (:file "_package_ExternalForce" :depends-on ("_package"))
    (:file "JointPosVel" :depends-on ("_package_JointPosVel"))
    (:file "_package_JointPosVel" :depends-on ("_package"))
    (:file "RobotMode" :depends-on ("_package_RobotMode"))
    (:file "_package_RobotMode" :depends-on ("_package"))
    (:file "RobotState" :depends-on ("_package_RobotState"))
    (:file "_package_RobotState" :depends-on ("_package"))
  ))