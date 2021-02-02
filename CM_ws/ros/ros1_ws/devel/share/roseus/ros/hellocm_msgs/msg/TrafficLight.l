;; Auto-generated. Do not edit!


(when (boundp 'hellocm_msgs::TrafficLight)
  (if (not (find-package "HELLOCM_MSGS"))
    (make-package "HELLOCM_MSGS"))
  (shadow 'TrafficLight (find-package "HELLOCM_MSGS")))
(unless (find-package "HELLOCM_MSGS::TRAFFICLIGHT")
  (make-package "HELLOCM_MSGS::TRAFFICLIGHT"))

(in-package "ROS")
;;//! \htmlinclude TrafficLight.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass hellocm_msgs::TrafficLight
  :super ros::object
  :slots (_header _time _cycleno _synthdelay _id _state _remain_time _remain_distance ))

(defmethod hellocm_msgs::TrafficLight
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:time __time) (instance ros::time :init))
    ((:cycleno __cycleno) 0)
    ((:synthdelay __synthdelay) 0.0)
    ((:id __id) 0.0)
    ((:state __state) 0.0)
    ((:remain_time __remain_time) 0.0)
    ((:remain_distance __remain_distance) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _time __time)
   (setq _cycleno (round __cycleno))
   (setq _synthdelay (float __synthdelay))
   (setq _id (float __id))
   (setq _state (float __state))
   (setq _remain_time (float __remain_time))
   (setq _remain_distance (float __remain_distance))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:cycleno
   (&optional __cycleno)
   (if __cycleno (setq _cycleno __cycleno)) _cycleno)
  (:synthdelay
   (&optional __synthdelay)
   (if __synthdelay (setq _synthdelay __synthdelay)) _synthdelay)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:remain_time
   (&optional __remain_time)
   (if __remain_time (setq _remain_time __remain_time)) _remain_time)
  (:remain_distance
   (&optional __remain_distance)
   (if __remain_distance (setq _remain_distance __remain_distance)) _remain_distance)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; time _time
    8
    ;; int64 _cycleno
    8
    ;; float64 _synthdelay
    8
    ;; float64 _id
    8
    ;; float64 _state
    8
    ;; float64 _remain_time
    8
    ;; float64 _remain_distance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; time _time
       (write-long (send _time :sec) s) (write-long (send _time :nsec) s)
     ;; int64 _cycleno
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _cycleno (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _cycleno) (= (length (_cycleno . bv)) 2)) ;; bignum
              (write-long (ash (elt (_cycleno . bv) 0) 0) s)
              (write-long (ash (elt (_cycleno . bv) 1) -1) s))
             ((and (class _cycleno) (= (length (_cycleno . bv)) 1)) ;; big1
              (write-long (elt (_cycleno . bv) 0) s)
              (write-long (if (>= _cycleno 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _cycleno s)(write-long (if (>= _cycleno 0) 0 #xffffffff) s)))
     ;; float64 _synthdelay
       (sys::poke _synthdelay (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _id
       (sys::poke _id (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _state
       (sys::poke _state (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _remain_time
       (sys::poke _remain_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _remain_distance
       (sys::poke _remain_distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; time _time
     (send _time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int64 _cycleno
#+(or :alpha :irix6 :x86_64)
      (setf _cycleno (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _cycleno (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _synthdelay
     (setq _synthdelay (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _id
     (setq _id (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _state
     (setq _state (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _remain_time
     (setq _remain_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _remain_distance
     (setq _remain_distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get hellocm_msgs::TrafficLight :md5sum-) "96a853204d29d897c2c8a468947a4a52")
(setf (get hellocm_msgs::TrafficLight :datatype-) "hellocm_msgs/TrafficLight")
(setf (get hellocm_msgs::TrafficLight :definition-)
      "#
# Message from CarMaker ROS Node to external ROS Node
#

# General
Header  header                                 # General ROS Header (optional)
time    time                                   # ROS time when message was sent (optional)
int64   cycleno                                # Cycle number since simulation start (optional)
float64 synthdelay                             # Synthetic delay to check synchronization (optional)

float64 id                                     # Signal ID
float64 state                                  # Signal State
float64 remain_time                            # Remain Time of Signal
float64 remain_distance                        # Distance between Traffic Signal and Vehicle Position 
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

")



(provide :hellocm_msgs/TrafficLight "96a853204d29d897c2c8a468947a4a52")


