;; Auto-generated. Do not edit!


(when (boundp 'hellocm_msgs::Ext2CM_Test)
  (if (not (find-package "HELLOCM_MSGS"))
    (make-package "HELLOCM_MSGS"))
  (shadow 'Ext2CM_Test (find-package "HELLOCM_MSGS")))
(unless (find-package "HELLOCM_MSGS::EXT2CM_TEST")
  (make-package "HELLOCM_MSGS::EXT2CM_TEST"))

(in-package "ROS")
;;//! \htmlinclude Ext2CM_Test.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass hellocm_msgs::Ext2CM_Test
  :super ros::object
  :slots (_header _cmd ))

(defmethod hellocm_msgs::Ext2CM_Test
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cmd __cmd) (instance hellocm_msgs::cmd :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _cmd __cmd)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:cmd
   (&rest __cmd)
   (if (keywordp (car __cmd))
       (send* _cmd __cmd)
     (progn
       (if __cmd (setq _cmd (car __cmd)))
       _cmd)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; hellocm_msgs/cmd _cmd
    (send _cmd :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; hellocm_msgs/cmd _cmd
       (send _cmd :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; hellocm_msgs/cmd _cmd
     (send _cmd :deserialize buf ptr-) (incf ptr- (send _cmd :serialization-length))
   ;;
   self)
  )

(setf (get hellocm_msgs::Ext2CM_Test :md5sum-) "cf8b01a8273bedcf082c4a0007472482")
(setf (get hellocm_msgs::Ext2CM_Test :datatype-) "hellocm_msgs/Ext2CM_Test")
(setf (get hellocm_msgs::Ext2CM_Test :definition-)
      "# General
Header  header                                 # General ROS Header (optional)

cmd cmd
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: hellocm_msgs/cmd
float64 linear_velocity
float64 linear_acceleration
float64 steering_angle
")



(provide :hellocm_msgs/Ext2CM_Test "cf8b01a8273bedcf082c4a0007472482")


