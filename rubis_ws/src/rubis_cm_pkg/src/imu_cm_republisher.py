#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pub = rospy.Publisher('imu_raw', Imu, queue_size=1)

def cb(msg):
    roll = msg.orientation.x
    pitch = msg.orientation.y
    yaw = msg.orientation.z*-1
    quat = quaternion_from_euler(roll, pitch, yaw)
    print(roll, pitch, yaw)
    
    out = msg
    out.header.stamp = rospy.Time.now()
    out.header.frame_id = 'imu'
    out.orientation.x = quat[0]
    out.orientation.y = quat[1]
    out.orientation.z = quat[2]
    out.orientation.w = quat[3]

    out.angular_velocity.z = out.angular_velocity.z * -1 

    pub.publish(out)


def main():
    rospy.init_node('imu_cm_republisher')
    sub = rospy.Subscriber('imu_out', Imu, cb)
    r = rospy.Rate(10)

    print("Start")
    rospy.spin()

if __name__ == '__main__':
    main()