from pynput import keyboard
import threading
import rospy
from std_msgs.msg import Float32
from hellocm_msgs.msg import Ext2CM_Test 

ACC_MIN = -10
ACC_MAX = 10
ACC_EMG = -100
PI = 3.141592
STEER_MIN = -3*PI
STEER_MAX = 3*PI

current_pressed = set()
global current_acc
global current_steer
current_acc = 0
current_steer = 0

def on_press(key):
    global current_acc
    global current_steer
    current_pressed.add(key)
    # print('Key %s pressed' % current_pressed)

    if keyboard.KeyCode(char='w') in current_pressed:
        if current_acc < ACC_MAX:
            current_acc += 1

    if keyboard.KeyCode(char='s') in current_pressed:
        if current_acc > ACC_MIN:
            current_acc += -5

    if keyboard.KeyCode(char='a') in current_pressed:
        if current_steer < STEER_MAX:
            current_steer += 0.5*PI/20

    if keyboard.KeyCode(char='d') in current_pressed:
        if current_steer > STEER_MIN:
            current_steer += -0.5*PI/20
    
    if keyboard.KeyCode(char='f') in current_pressed:
        current_steer = 0
        current_acc = ACC_EMG

    # -10/57.3

    # if keyboard.KeyCode(char='w') not in current_pressed:
    #     if keyboard.KeyCode(char='s') not in current_pressed:
    #         current_acc = -1
    #         current_steer = 0
    # print('acc : %f steer : %f' %(current_acc, current_steer))

def keyboard_routine():
    with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
        listener.join()

def on_release(key):
    global current_acc
    global current_steer
    if key == keyboard.Key.backspace:
        print('\nYou pressed backspace. Quit!')
        return False
    if key == keyboard.KeyCode(char='a'):
        current_steer = 0        
    if key == keyboard.KeyCode(char='d'):
        current_steer = 0        
    if key == keyboard.KeyCode(char='w'):
        current_acc = -1        
    if key == keyboard.KeyCode(char='s'):
        current_acc = -1
    if key == keyboard.KeyCode(char='f'):
        current_acc = -1      
    if key in current_pressed:
        current_pressed.remove(key)

if __name__ == '__main__':
    keyboard_thread = threading.Thread(target=keyboard_routine)
    keyboard_thread.start()

    ext_pub = rospy.Publisher('ctrl_cmd', Ext2CM_Test, queue_size=10)
    # steer_pub = rospy.Publisher('test_steer', Float32, queue_size=10)

    rospy.init_node('keyboard_test')
    rate = rospy.Rate(100)

    ext_msg = Ext2CM_Test()
    
    while not rospy.is_shutdown():
        ext_msg.cmd.linear_acceleration = current_acc
        ext_msg.cmd.steering_angle = current_steer

        ext_pub.publish(ext_msg)

        rate.sleep()