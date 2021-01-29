;; Auto-generated. Do not edit!


(when (boundp 'hellocm_msgs::cmd)
  (if (not (find-package "HELLOCM_MSGS"))
    (make-package "HELLOCM_MSGS"))
  (shadow 'cmd (find-package "HELLOCM_MSGS")))
(unless (find-package "HELLOCM_MSGS::CMD")
  (make-package "HELLOCM_MSGS::CMD"))

(in-package "ROS")
;;//! \htmlinclude cmd.msg.html


(defclass hellocm_msgs::cmd
  :super ros::object
  :slots (_linear_velocity _linear_acceleration _steering_angle ))

(defmethod hellocm_msgs::cmd
  (:init
   (&key
    ((:linear_velocity __linear_velocity) 0.0)
    ((:linear_acceleration __linear_acceleration) 0.0)
    ((:steering_angle __steering_angle) 0.0)
    )
   (send-super :init)
   (setq _linear_velocity (float __linear_velocity))
   (setq _linear_acceleration (float __linear_acceleration))
   (setq _steering_angle (float __steering_angle))
   self)
  (:linear_velocity
   (&optional __linear_velocity)
   (if __linear_velocity (setq _linear_velocity __linear_velocity)) _linear_velocity)
  (:linear_acceleration
   (&optional __linear_acceleration)
   (if __linear_acceleration (setq _linear_acceleration __linear_acceleration)) _linear_acceleration)
  (:steering_angle
   (&optional __steering_angle)
   (if __steering_angle (setq _steering_angle __steering_angle)) _steering_angle)
  (:serialization-length
   ()
   (+
    ;; float64 _linear_velocity
    8
    ;; float64 _linear_acceleration
    8
    ;; float64 _steering_angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _linear_velocity
       (sys::poke _linear_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _linear_acceleration
       (sys::poke _linear_acceleration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steering_angle
       (sys::poke _steering_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _linear_velocity
     (setq _linear_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _linear_acceleration
     (setq _linear_acceleration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steering_angle
     (setq _steering_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get hellocm_msgs::cmd :md5sum-) "97bdb8c6dda03d6c6f5a1a98c32d26b8")
(setf (get hellocm_msgs::cmd :datatype-) "hellocm_msgs/cmd")
(setf (get hellocm_msgs::cmd :definition-)
      "float64 linear_velocity
float64 linear_acceleration
float64 steering_angle
")



(provide :hellocm_msgs/cmd "97bdb8c6dda03d6c6f5a1a98c32d26b8")


