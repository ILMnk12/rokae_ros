;; Auto-generated. Do not edit!


(when (boundp 'rokae_msgs::JointPosVel)
  (if (not (find-package "ROKAE_MSGS"))
    (make-package "ROKAE_MSGS"))
  (shadow 'JointPosVel (find-package "ROKAE_MSGS")))
(unless (find-package "ROKAE_MSGS::JOINTPOSVEL")
  (make-package "ROKAE_MSGS::JOINTPOSVEL"))

(in-package "ROS")
;;//! \htmlinclude JointPosVel.msg.html


(defclass rokae_msgs::JointPosVel
  :super ros::object
  :slots (_timestamp _positions _velocities ))

(defmethod rokae_msgs::JointPosVel
  (:init
   (&key
    ((:timestamp __timestamp) 0.0)
    ((:positions __positions) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocities __velocities) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _timestamp (float __timestamp))
   (setq _positions __positions)
   (setq _velocities __velocities)
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:positions
   (&optional __positions)
   (if __positions (setq _positions __positions)) _positions)
  (:velocities
   (&optional __velocities)
   (if __velocities (setq _velocities __velocities)) _velocities)
  (:serialization-length
   ()
   (+
    ;; float64 _timestamp
    8
    ;; float64[] _positions
    (* 8    (length _positions)) 4
    ;; float64[] _velocities
    (* 8    (length _velocities)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _positions
     (write-long (length _positions) s)
     (dotimes (i (length _positions))
       (sys::poke (elt _positions i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocities
     (write-long (length _velocities) s)
     (dotimes (i (length _velocities))
       (sys::poke (elt _velocities i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _positions i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocities i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get rokae_msgs::JointPosVel :md5sum-) "deb4fb4efd7a8f0c7867baee56f1a9a5")
(setf (get rokae_msgs::JointPosVel :datatype-) "rokae_msgs/JointPosVel")
(setf (get rokae_msgs::JointPosVel :definition-)
      "# This is for joint postion and velocity accquire.
float64 timestamp
float64[] positions
float64[] velocities
")



(provide :rokae_msgs/JointPosVel "deb4fb4efd7a8f0c7867baee56f1a9a5")


