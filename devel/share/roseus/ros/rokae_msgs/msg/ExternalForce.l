;; Auto-generated. Do not edit!


(when (boundp 'rokae_msgs::ExternalForce)
  (if (not (find-package "ROKAE_MSGS"))
    (make-package "ROKAE_MSGS"))
  (shadow 'ExternalForce (find-package "ROKAE_MSGS")))
(unless (find-package "ROKAE_MSGS::EXTERNALFORCE")
  (make-package "ROKAE_MSGS::EXTERNALFORCE"))

(in-package "ROS")
;;//! \htmlinclude ExternalForce.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass rokae_msgs::ExternalForce
  :super ros::object
  :slots (_force _moment ))

(defmethod rokae_msgs::ExternalForce
  (:init
   (&key
    ((:force __force) (instance geometry_msgs::Vector3 :init))
    ((:moment __moment) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _force __force)
   (setq _moment __moment)
   self)
  (:force
   (&rest __force)
   (if (keywordp (car __force))
       (send* _force __force)
     (progn
       (if __force (setq _force (car __force)))
       _force)))
  (:moment
   (&rest __moment)
   (if (keywordp (car __moment))
       (send* _moment __moment)
     (progn
       (if __moment (setq _moment (car __moment)))
       _moment)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _force
    (send _force :serialization-length)
    ;; geometry_msgs/Vector3 _moment
    (send _moment :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _force
       (send _force :serialize s)
     ;; geometry_msgs/Vector3 _moment
       (send _moment :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _force
     (send _force :deserialize buf ptr-) (incf ptr- (send _force :serialization-length))
   ;; geometry_msgs/Vector3 _moment
     (send _moment :deserialize buf ptr-) (incf ptr- (send _moment :serialization-length))
   ;;
   self)
  )

(setf (get rokae_msgs::ExternalForce :md5sum-) "572f71e60f73d6a9e5c0b5d4c329a492")
(setf (get rokae_msgs::ExternalForce :datatype-) "rokae_msgs/ExternalForce")
(setf (get rokae_msgs::ExternalForce :definition-)
      "# force vector separated into force and moment
geometry_msgs/Vector3 force
geometry_msgs/Vector3 moment
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :rokae_msgs/ExternalForce "572f71e60f73d6a9e5c0b5d4c329a492")


