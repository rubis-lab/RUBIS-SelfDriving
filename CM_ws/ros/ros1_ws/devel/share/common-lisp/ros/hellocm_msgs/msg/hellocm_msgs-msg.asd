
(cl:in-package :asdf)

(defsystem "hellocm_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CM2Ext" :depends-on ("_package_CM2Ext"))
    (:file "_package_CM2Ext" :depends-on ("_package"))
    (:file "CM2Ext_Test" :depends-on ("_package_CM2Ext_Test"))
    (:file "_package_CM2Ext_Test" :depends-on ("_package"))
    (:file "Ext2CM" :depends-on ("_package_Ext2CM"))
    (:file "_package_Ext2CM" :depends-on ("_package"))
    (:file "Ext2CM_Test" :depends-on ("_package_Ext2CM_Test"))
    (:file "_package_Ext2CM_Test" :depends-on ("_package"))
    (:file "GPS_Out" :depends-on ("_package_GPS_Out"))
    (:file "_package_GPS_Out" :depends-on ("_package"))
    (:file "Speed_Limit" :depends-on ("_package_Speed_Limit"))
    (:file "_package_Speed_Limit" :depends-on ("_package"))
    (:file "TrafficLight" :depends-on ("_package_TrafficLight"))
    (:file "_package_TrafficLight" :depends-on ("_package"))
    (:file "cmd" :depends-on ("_package_cmd"))
    (:file "_package_cmd" :depends-on ("_package"))
  ))