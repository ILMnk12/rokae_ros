; Auto-generated. Do not edit!


(cl:in-package rokae_msgs-msg)


;//! \htmlinclude RobotMode.msg.html

(cl:defclass <RobotMode> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:float
    :initform 0.0)
   (is_robot_connected
    :reader is_robot_connected
    :initarg :is_robot_connected
    :type cl:boolean
    :initform cl:nil)
   (is_real_robot_enabled
    :reader is_real_robot_enabled
    :initarg :is_real_robot_enabled
    :type cl:boolean
    :initform cl:nil)
   (is_robot_power_on
    :reader is_robot_power_on
    :initarg :is_robot_power_on
    :type cl:boolean
    :initform cl:nil)
   (is_robot_running
    :reader is_robot_running
    :initarg :is_robot_running
    :type cl:boolean
    :initform cl:nil)
   (is_program_running
    :reader is_program_running
    :initarg :is_program_running
    :type cl:boolean
    :initform cl:nil)
   (is_emergency_stopped
    :reader is_emergency_stopped
    :initarg :is_emergency_stopped
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RobotMode (<RobotMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokae_msgs-msg:<RobotMode> is deprecated: use rokae_msgs-msg:RobotMode instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <RobotMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_msgs-msg:timestamp-val is deprecated.  Use rokae_msgs-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'is_robot_connected-val :lambda-list '(m))
(cl:defmethod is_robot_connected-val ((m <RobotMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_msgs-msg:is_robot_connected-val is deprecated.  Use rokae_msgs-msg:is_robot_connected instead.")
  (is_robot_connected m))

(cl:ensure-generic-function 'is_real_robot_enabled-val :lambda-list '(m))
(cl:defmethod is_real_robot_enabled-val ((m <RobotMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_msgs-msg:is_real_robot_enabled-val is deprecated.  Use rokae_msgs-msg:is_real_robot_enabled instead.")
  (is_real_robot_enabled m))

(cl:ensure-generic-function 'is_robot_power_on-val :lambda-list '(m))
(cl:defmethod is_robot_power_on-val ((m <RobotMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_msgs-msg:is_robot_power_on-val is deprecated.  Use rokae_msgs-msg:is_robot_power_on instead.")
  (is_robot_power_on m))

(cl:ensure-generic-function 'is_robot_running-val :lambda-list '(m))
(cl:defmethod is_robot_running-val ((m <RobotMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_msgs-msg:is_robot_running-val is deprecated.  Use rokae_msgs-msg:is_robot_running instead.")
  (is_robot_running m))

(cl:ensure-generic-function 'is_program_running-val :lambda-list '(m))
(cl:defmethod is_program_running-val ((m <RobotMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_msgs-msg:is_program_running-val is deprecated.  Use rokae_msgs-msg:is_program_running instead.")
  (is_program_running m))

(cl:ensure-generic-function 'is_emergency_stopped-val :lambda-list '(m))
(cl:defmethod is_emergency_stopped-val ((m <RobotMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_msgs-msg:is_emergency_stopped-val is deprecated.  Use rokae_msgs-msg:is_emergency_stopped instead.")
  (is_emergency_stopped m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotMode>) ostream)
  "Serializes a message object of type '<RobotMode>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timestamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_robot_connected) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_real_robot_enabled) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_robot_power_on) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_robot_running) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_program_running) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_emergency_stopped) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotMode>) istream)
  "Deserializes a message object of type '<RobotMode>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timestamp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'is_robot_connected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_real_robot_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_robot_power_on) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_robot_running) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_program_running) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_emergency_stopped) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotMode>)))
  "Returns string type for a message object of type '<RobotMode>"
  "rokae_msgs/RobotMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMode)))
  "Returns string type for a message object of type 'RobotMode"
  "rokae_msgs/RobotMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotMode>)))
  "Returns md5sum for a message object of type '<RobotMode>"
  "37b92b84717583128a825ed4248ac20e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotMode)))
  "Returns md5sum for a message object of type 'RobotMode"
  "37b92b84717583128a825ed4248ac20e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotMode>)))
  "Returns full string definition for message of type '<RobotMode>"
  (cl:format cl:nil "# This is for robot state mode structure~%float64 timestamp~%bool is_robot_connected~%bool is_real_robot_enabled~%bool is_robot_power_on~%bool is_robot_running~%bool is_program_running~%bool is_emergency_stopped~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotMode)))
  "Returns full string definition for message of type 'RobotMode"
  (cl:format cl:nil "# This is for robot state mode structure~%float64 timestamp~%bool is_robot_connected~%bool is_real_robot_enabled~%bool is_robot_power_on~%bool is_robot_running~%bool is_program_running~%bool is_emergency_stopped~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotMode>))
  (cl:+ 0
     8
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotMode>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotMode
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':is_robot_connected (is_robot_connected msg))
    (cl:cons ':is_real_robot_enabled (is_real_robot_enabled msg))
    (cl:cons ':is_robot_power_on (is_robot_power_on msg))
    (cl:cons ':is_robot_running (is_robot_running msg))
    (cl:cons ':is_program_running (is_program_running msg))
    (cl:cons ':is_emergency_stopped (is_emergency_stopped msg))
))
