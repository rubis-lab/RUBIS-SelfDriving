; Auto-generated. Do not edit!


(cl:in-package hellocm_msgs-msg)


;//! \htmlinclude CM2Ext_Test.msg.html

(cl:defclass <CM2Ext_Test> (roslisp-msg-protocol:ros-message)
  ((test1
    :reader test1
    :initarg :test1
    :type cl:float
    :initform 0.0))
)

(cl:defclass CM2Ext_Test (<CM2Ext_Test>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CM2Ext_Test>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CM2Ext_Test)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hellocm_msgs-msg:<CM2Ext_Test> is deprecated: use hellocm_msgs-msg:CM2Ext_Test instead.")))

(cl:ensure-generic-function 'test1-val :lambda-list '(m))
(cl:defmethod test1-val ((m <CM2Ext_Test>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hellocm_msgs-msg:test1-val is deprecated.  Use hellocm_msgs-msg:test1 instead.")
  (test1 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CM2Ext_Test>) ostream)
  "Serializes a message object of type '<CM2Ext_Test>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'test1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CM2Ext_Test>) istream)
  "Deserializes a message object of type '<CM2Ext_Test>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'test1) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CM2Ext_Test>)))
  "Returns string type for a message object of type '<CM2Ext_Test>"
  "hellocm_msgs/CM2Ext_Test")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CM2Ext_Test)))
  "Returns string type for a message object of type 'CM2Ext_Test"
  "hellocm_msgs/CM2Ext_Test")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CM2Ext_Test>)))
  "Returns md5sum for a message object of type '<CM2Ext_Test>"
  "1544912b023f841df84f892fda2b3c07")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CM2Ext_Test)))
  "Returns md5sum for a message object of type 'CM2Ext_Test"
  "1544912b023f841df84f892fda2b3c07")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CM2Ext_Test>)))
  "Returns full string definition for message of type '<CM2Ext_Test>"
  (cl:format cl:nil "float64 test1~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CM2Ext_Test)))
  "Returns full string definition for message of type 'CM2Ext_Test"
  (cl:format cl:nil "float64 test1~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CM2Ext_Test>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CM2Ext_Test>))
  "Converts a ROS message object to a list"
  (cl:list 'CM2Ext_Test
    (cl:cons ':test1 (test1 msg))
))
