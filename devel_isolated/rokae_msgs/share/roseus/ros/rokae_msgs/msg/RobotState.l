;; Auto-generated. Do not edit!


(when (boundp 'rokae_msgs::RobotState)
  (if (not (find-package "ROKAE_MSGS"))
    (make-package "ROKAE_MSGS"))
  (shadow 'RobotState (find-package "ROKAE_MSGS")))
(unless (find-package "ROKAE_MSGS::ROBOTSTATE")
  (make-package "ROKAE_MSGS::ROBOTSTATE"))

(in-package "ROS")
;;//! \htmlinclude RobotState.msg.html


(defclass rokae_msgs::RobotState
  :super ros::object
  :slots (_timestamp _joint_pose _joint_velocity _joint_cmd_acceleration _joint_torque _joint_filter_torque _arm_angle ))

(defmethod rokae_msgs::RobotState
  (:init
   (&key
    ((:timestamp __timestamp) 0.0)
    ((:joint_pose __joint_pose) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_velocity __joint_velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_cmd_acceleration __joint_cmd_acceleration) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_torque __joint_torque) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_filter_torque __joint_filter_torque) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:arm_angle __arm_angle) 0.0)
    )
   (send-super :init)
   (setq _timestamp (float __timestamp))
   (setq _joint_pose __joint_pose)
   (setq _joint_velocity __joint_velocity)
   (setq _joint_cmd_acceleration __joint_cmd_acceleration)
   (setq _joint_torque __joint_torque)
   (setq _joint_filter_torque __joint_filter_torque)
   (setq _arm_angle (float __arm_angle))
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:joint_pose
   (&optional __joint_pose)
   (if __joint_pose (setq _joint_pose __joint_pose)) _joint_pose)
  (:joint_velocity
   (&optional __joint_velocity)
   (if __joint_velocity (setq _joint_velocity __joint_velocity)) _joint_velocity)
  (:joint_cmd_acceleration
   (&optional __joint_cmd_acceleration)
   (if __joint_cmd_acceleration (setq _joint_cmd_acceleration __joint_cmd_acceleration)) _joint_cmd_acceleration)
  (:joint_torque
   (&optional __joint_torque)
   (if __joint_torque (setq _joint_torque __joint_torque)) _joint_torque)
  (:joint_filter_torque
   (&optional __joint_filter_torque)
   (if __joint_filter_torque (setq _joint_filter_torque __joint_filter_torque)) _joint_filter_torque)
  (:arm_angle
   (&optional __arm_angle)
   (if __arm_angle (setq _arm_angle __arm_angle)) _arm_angle)
  (:serialization-length
   ()
   (+
    ;; float64 _timestamp
    8
    ;; float64[] _joint_pose
    (* 8    (length _joint_pose)) 4
    ;; float64[] _joint_velocity
    (* 8    (length _joint_velocity)) 4
    ;; float64[] _joint_cmd_acceleration
    (* 8    (length _joint_cmd_acceleration)) 4
    ;; float64[] _joint_torque
    (* 8    (length _joint_torque)) 4
    ;; float64[] _joint_filter_torque
    (* 8    (length _joint_filter_torque)) 4
    ;; float64 _arm_angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _joint_pose
     (write-long (length _joint_pose) s)
     (dotimes (i (length _joint_pose))
       (sys::poke (elt _joint_pose i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_velocity
     (write-long (length _joint_velocity) s)
     (dotimes (i (length _joint_velocity))
       (sys::poke (elt _joint_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_cmd_acceleration
     (write-long (length _joint_cmd_acceleration) s)
     (dotimes (i (length _joint_cmd_acceleration))
       (sys::poke (elt _joint_cmd_acceleration i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_torque
     (write-long (length _joint_torque) s)
     (dotimes (i (length _joint_torque))
       (sys::poke (elt _joint_torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_filter_torque
     (write-long (length _joint_filter_torque) s)
     (dotimes (i (length _joint_filter_torque))
       (sys::poke (elt _joint_filter_torque i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _arm_angle
       (sys::poke _arm_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _joint_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_pose (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_pose i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_cmd_acceleration
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_cmd_acceleration (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_cmd_acceleration i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_torque
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_torque (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_filter_torque
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_filter_torque (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_filter_torque i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _arm_angle
     (setq _arm_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get rokae_msgs::RobotState :md5sum-) "bd80e953e8359840e704f48ebdde5ced")
(setf (get rokae_msgs::RobotState :datatype-) "rokae_msgs/RobotState")
(setf (get rokae_msgs::RobotState :definition-)
      "# Data structure for robot status description

# timestamp
float64 timestamp                    # time stamp   

# joint status
float64[] joint_pose                 # joint_pose
float64[] joint_velocity             # joint_velocity
float64[] joint_cmd_acceleration         # joint_acceleration
float64[] joint_torque               # joint_torque
float64[] joint_filter_torque        # joint_filter_torque

# arm angle state
float64 arm_angle                    # arm angle
")



(provide :rokae_msgs/RobotState "bd80e953e8359840e704f48ebdde5ced")


