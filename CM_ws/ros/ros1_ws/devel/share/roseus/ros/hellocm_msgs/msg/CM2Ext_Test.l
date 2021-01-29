;; Auto-generated. Do not edit!


(when (boundp 'hellocm_msgs::CM2Ext_Test)
  (if (not (find-package "HELLOCM_MSGS"))
    (make-package "HELLOCM_MSGS"))
  (shadow 'CM2Ext_Test (find-package "HELLOCM_MSGS")))
(unless (find-package "HELLOCM_MSGS::CM2EXT_TEST")
  (make-package "HELLOCM_MSGS::CM2EXT_TEST"))

(in-package "ROS")
;;//! \htmlinclude CM2Ext_Test.msg.html


(defclass hellocm_msgs::CM2Ext_Test
  :super ros::object
  :slots (_test1 ))

(defmethod hellocm_msgs::CM2Ext_Test
  (:init
   (&key
    ((:test1 __test1) 0.0)
    )
   (send-super :init)
   (setq _test1 (float __test1))
   self)
  (:test1
   (&optional __test1)
   (if __test1 (setq _test1 __test1)) _test1)
  (:serialization-length
   ()
   (+
    ;; float64 _test1
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _test1
       (sys::poke _test1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _test1
     (setq _test1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get hellocm_msgs::CM2Ext_Test :md5sum-) "1544912b023f841df84f892fda2b3c07")
(setf (get hellocm_msgs::CM2Ext_Test :datatype-) "hellocm_msgs/CM2Ext_Test")
(setf (get hellocm_msgs::CM2Ext_Test :definition-)
      "float64 test1
")



(provide :hellocm_msgs/CM2Ext_Test "1544912b023f841df84f892fda2b3c07")


