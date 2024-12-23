;; Auto-generated. Do not edit!


(when (boundp 'rokae_msgs::RobotMode)
  (if (not (find-package "ROKAE_MSGS"))
    (make-package "ROKAE_MSGS"))
  (shadow 'RobotMode (find-package "ROKAE_MSGS")))
(unless (find-package "ROKAE_MSGS::ROBOTMODE")
  (make-package "ROKAE_MSGS::ROBOTMODE"))

(in-package "ROS")
;;//! \htmlinclude RobotMode.msg.html


(defclass rokae_msgs::RobotMode
  :super ros::object
  :slots (_timestamp _is_robot_connected _is_real_robot_enabled _is_robot_power_on _is_robot_running _is_program_running _is_emergency_stopped ))

(defmethod rokae_msgs::RobotMode
  (:init
   (&key
    ((:timestamp __timestamp) 0.0)
    ((:is_robot_connected __is_robot_connected) nil)
    ((:is_real_robot_enabled __is_real_robot_enabled) nil)
    ((:is_robot_power_on __is_robot_power_on) nil)
    ((:is_robot_running __is_robot_running) nil)
    ((:is_program_running __is_program_running) nil)
    ((:is_emergency_stopped __is_emergency_stopped) nil)
    )
   (send-super :init)
   (setq _timestamp (float __timestamp))
   (setq _is_robot_connected __is_robot_connected)
   (setq _is_real_robot_enabled __is_real_robot_enabled)
   (setq _is_robot_power_on __is_robot_power_on)
   (setq _is_robot_running __is_robot_running)
   (setq _is_program_running __is_program_running)
   (setq _is_emergency_stopped __is_emergency_stopped)
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:is_robot_connected
   (&optional (__is_robot_connected :null))
   (if (not (eq __is_robot_connected :null)) (setq _is_robot_connected __is_robot_connected)) _is_robot_connected)
  (:is_real_robot_enabled
   (&optional (__is_real_robot_enabled :null))
   (if (not (eq __is_real_robot_enabled :null)) (setq _is_real_robot_enabled __is_real_robot_enabled)) _is_real_robot_enabled)
  (:is_robot_power_on
   (&optional (__is_robot_power_on :null))
   (if (not (eq __is_robot_power_on :null)) (setq _is_robot_power_on __is_robot_power_on)) _is_robot_power_on)
  (:is_robot_running
   (&optional (__is_robot_running :null))
   (if (not (eq __is_robot_running :null)) (setq _is_robot_running __is_robot_running)) _is_robot_running)
  (:is_program_running
   (&optional (__is_program_running :null))
   (if (not (eq __is_program_running :null)) (setq _is_program_running __is_program_running)) _is_program_running)
  (:is_emergency_stopped
   (&optional (__is_emergency_stopped :null))
   (if (not (eq __is_emergency_stopped :null)) (setq _is_emergency_stopped __is_emergency_stopped)) _is_emergency_stopped)
  (:serialization-length
   ()
   (+
    ;; float64 _timestamp
    8
    ;; bool _is_robot_connected
    1
    ;; bool _is_real_robot_enabled
    1
    ;; bool _is_robot_power_on
    1
    ;; bool _is_robot_running
    1
    ;; bool _is_program_running
    1
    ;; bool _is_emergency_stopped
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _is_robot_connected
       (if _is_robot_connected (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_real_robot_enabled
       (if _is_real_robot_enabled (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_robot_power_on
       (if _is_robot_power_on (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_robot_running
       (if _is_robot_running (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_program_running
       (if _is_program_running (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_emergency_stopped
       (if _is_emergency_stopped (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _is_robot_connected
     (setq _is_robot_connected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_real_robot_enabled
     (setq _is_real_robot_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_robot_power_on
     (setq _is_robot_power_on (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_robot_running
     (setq _is_robot_running (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_program_running
     (setq _is_program_running (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_emergency_stopped
     (setq _is_emergency_stopped (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get rokae_msgs::RobotMode :md5sum-) "37b92b84717583128a825ed4248ac20e")
(setf (get rokae_msgs::RobotMode :datatype-) "rokae_msgs/RobotMode")
(setf (get rokae_msgs::RobotMode :definition-)
      "# This is for robot state mode structure
float64 timestamp
bool is_robot_connected
bool is_real_robot_enabled
bool is_robot_power_on
bool is_robot_running
bool is_program_running
bool is_emergency_stopped
")



(provide :rokae_msgs/RobotMode "37b92b84717583128a825ed4248ac20e")


