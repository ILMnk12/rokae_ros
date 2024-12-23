; Auto-generated. Do not edit!


(cl:in-package rokae_msgs-msg)


;//! \htmlinclude ExternalForce.msg.html

(cl:defclass <ExternalForce> (roslisp-msg-protocol:ros-message)
  ((force
    :reader force
    :initarg :force
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (moment
    :reader moment
    :initarg :moment
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass ExternalForce (<ExternalForce>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExternalForce>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExternalForce)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokae_msgs-msg:<ExternalForce> is deprecated: use rokae_msgs-msg:ExternalForce instead.")))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <ExternalForce>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_msgs-msg:force-val is deprecated.  Use rokae_msgs-msg:force instead.")
  (force m))

(cl:ensure-generic-function 'moment-val :lambda-list '(m))
(cl:defmethod moment-val ((m <ExternalForce>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_msgs-msg:moment-val is deprecated.  Use rokae_msgs-msg:moment instead.")
  (moment m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExternalForce>) ostream)
  "Serializes a message object of type '<ExternalForce>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'moment) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExternalForce>) istream)
  "Deserializes a message object of type '<ExternalForce>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'moment) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExternalForce>)))
  "Returns string type for a message object of type '<ExternalForce>"
  "rokae_msgs/ExternalForce")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExternalForce)))
  "Returns string type for a message object of type 'ExternalForce"
  "rokae_msgs/ExternalForce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExternalForce>)))
  "Returns md5sum for a message object of type '<ExternalForce>"
  "572f71e60f73d6a9e5c0b5d4c329a492")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExternalForce)))
  "Returns md5sum for a message object of type 'ExternalForce"
  "572f71e60f73d6a9e5c0b5d4c329a492")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExternalForce>)))
  "Returns full string definition for message of type '<ExternalForce>"
  (cl:format cl:nil "# force vector separated into force and moment~%geometry_msgs/Vector3 force~%geometry_msgs/Vector3 moment~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExternalForce)))
  "Returns full string definition for message of type 'ExternalForce"
  (cl:format cl:nil "# force vector separated into force and moment~%geometry_msgs/Vector3 force~%geometry_msgs/Vector3 moment~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExternalForce>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'moment))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExternalForce>))
  "Converts a ROS message object to a list"
  (cl:list 'ExternalForce
    (cl:cons ':force (force msg))
    (cl:cons ':moment (moment msg))
))
