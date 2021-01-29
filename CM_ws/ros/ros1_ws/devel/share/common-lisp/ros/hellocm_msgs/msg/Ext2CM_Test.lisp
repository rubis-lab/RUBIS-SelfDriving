; Auto-generated. Do not edit!


(cl:in-package hellocm_msgs-msg)


;//! \htmlinclude Ext2CM_Test.msg.html

(cl:defclass <Ext2CM_Test> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cmd
    :reader cmd
    :initarg :cmd
    :type hellocm_msgs-msg:cmd
    :initform (cl:make-instance 'hellocm_msgs-msg:cmd)))
)

(cl:defclass Ext2CM_Test (<Ext2CM_Test>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ext2CM_Test>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ext2CM_Test)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hellocm_msgs-msg:<Ext2CM_Test> is deprecated: use hellocm_msgs-msg:Ext2CM_Test instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Ext2CM_Test>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hellocm_msgs-msg:header-val is deprecated.  Use hellocm_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <Ext2CM_Test>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hellocm_msgs-msg:cmd-val is deprecated.  Use hellocm_msgs-msg:cmd instead.")
  (cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ext2CM_Test>) ostream)
  "Serializes a message object of type '<Ext2CM_Test>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cmd) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ext2CM_Test>) istream)
  "Deserializes a message object of type '<Ext2CM_Test>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cmd) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ext2CM_Test>)))
  "Returns string type for a message object of type '<Ext2CM_Test>"
  "hellocm_msgs/Ext2CM_Test")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ext2CM_Test)))
  "Returns string type for a message object of type 'Ext2CM_Test"
  "hellocm_msgs/Ext2CM_Test")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ext2CM_Test>)))
  "Returns md5sum for a message object of type '<Ext2CM_Test>"
  "cf8b01a8273bedcf082c4a0007472482")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ext2CM_Test)))
  "Returns md5sum for a message object of type 'Ext2CM_Test"
  "cf8b01a8273bedcf082c4a0007472482")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ext2CM_Test>)))
  "Returns full string definition for message of type '<Ext2CM_Test>"
  (cl:format cl:nil "# General~%Header  header                                 # General ROS Header (optional)~%~%cmd cmd~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: hellocm_msgs/cmd~%float64 linear_velocity~%float64 linear_acceleration~%float64 steering_angle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ext2CM_Test)))
  "Returns full string definition for message of type 'Ext2CM_Test"
  (cl:format cl:nil "# General~%Header  header                                 # General ROS Header (optional)~%~%cmd cmd~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: hellocm_msgs/cmd~%float64 linear_velocity~%float64 linear_acceleration~%float64 steering_angle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ext2CM_Test>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cmd))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ext2CM_Test>))
  "Converts a ROS message object to a list"
  (cl:list 'Ext2CM_Test
    (cl:cons ':header (header msg))
    (cl:cons ':cmd (cmd msg))
))
