'''
/*******************************************************************************
 * FileName:       top_level.py
 * Company:        Tennessee Tech University
 *
 * Software License Agreement
 *
 * The software supplied herewith by Tennessee Tech University.
 * The software is owned by Tennessee Tech University, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN "AS-IS" CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO,
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE APPLY TO THIS SOFTWARE. TENNESSEE TECH UNIVERSITY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author                Date                  Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Nathan Gardner        3/18/23               Original
 * Luke McGill           3/18/23               Original
 * Not Mark or Madison
 ******************************************************************************/
'''

from time import sleep

import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def pub_direction(x, y, z):
    cmd_vel_msg.linear.x = x
    cmd_vel_msg.linear.y = y
    cmd_vel_msg.angular.z = z
    cmd_vel_pub.publish(cmd_vel_msg)

def extend_trailer():
    cmd_solenoid_pos_msg.data = True
    cmd_solenoid_pos_pub.publish(cmd_solenoid_pos_msg)
    sleep(1)
    cmd_servo_pos_msg.data = "EXTEND"
    cmd_servo_pos_pub.publish(cmd_servo_pos_msg)
    sleep(1)
    cmd_solenoid_pos_msg.data = False
    cmd_solenoid_pos_pub.publish(cmd_solenoid_pos_msg)

def turn_right():
    pub_direction(0,0,0)
    sleep(0.5)
    global fr_encoder
    fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pos = fr_encoder
    while(abs(pos - fr_encoder) < 780):
        pub_direction(0,0,50)
        sleep(0.001)
        fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pub_direction(0,0,0)
    sleep(0.5)


def forward(clicks, speed):
    global fr_encoder
    fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pos = fr_encoder
    while(abs(pos - fr_encoder) < clicks):
        pub_direction(0,speed,0)
        sleep(0.001)
        fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pub_direction(0,0,0)

def backward(clicks, speed):
    global fr_encoder
    fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pos = fr_encoder
    while(abs(pos - fr_encoder) < clicks):
        pub_direction(0,-speed,0)
        sleep(0.001)
        fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pub_direction(0,0,0)

def left(clicks, speed):
    global encoder
    encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None)
    fl_pos = encoder.data[0]
    fr_pos = encoder.data[1]
    rl_pos = encoder.data[2]
    rr_pos = encoder.data[3]
    while((abs(fl_pos - encoder.data[0]) < clicks) and (abs(fr_pos - encoder.data[1]) < clicks) and (abs(rl_pos - encoder.data[2]) < clicks) and (abs(rr_pos - encoder.data[3]) < clicks)):
        pub_direction(-speed,0,0)
        sleep(0.001)
        encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None)
    pub_direction(0,0,0)

def spin(clicks):
    global encoder
    encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None)
    fl_pos = encoder.data[0]
    fr_pos = encoder.data[1]
    rl_pos = encoder.data[2]
    rr_pos = encoder.data[3]
    while((abs(fl_pos - encoder.data[0]) < clicks) and (abs(fr_pos - encoder.data[1]) < clicks) and (abs(rl_pos - encoder.data[2]) < clicks) and (abs(rr_pos - encoder.data[3]) < clicks)):
        pub_direction(50,0,0)
        sleep(0.001)
        encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None)
    pub_direction(0,0,0)


def right(clicks, speed):
    global fr_encoder
    fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pos = fr_encoder
    while(abs(pos - fr_encoder) < clicks):
        pub_direction(speed,0,0)
        sleep(0.001)
        fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pub_direction(0,0,0)

def callback(data):
    print(str(data.data[0]) + ' ' + str(data.data[1]) + ' ' + str(data.data[2]) + ' ' + str(data.data[3]))
    fl_encoder = data.data[0]
    fr_encoder = data.data[1]
    rl_encoder = data.data[2]
    rr_encoder = data.data[3]

# not needed 
def start_callback(data):
    pass

# initialize things
fl_encoder = 0
fr_encoder = 0
rl_encoder = 0
rr_encoder = 0

global cmd_vel_msg
global cmd_servo_pos_msg
global cmd_solenoid_pos_msg
cmd_vel_msg = Twist()
cmd_servo_pos_msg = String()
cmd_solenoid_pos_msg = Bool()

# initislize publishers and subscribers 
cmd_vel_pub = rospy.Publisher('/locomotion/cmd_vel', Twist, queue_size=10)
cmd_servo_pos_pub = rospy.Publisher('/duckstorage/cmd_servo_pos', String, queue_size=10)
cmd_solenoid_pos_pub = rospy.Publisher('/duckstorage/cmd_solenoid_pos', Bool, queue_size=10)
sub = rospy.Subscriber('/locomotion/encoder', Int32MultiArray, callback)
start_sub = rospy.Subscriber('/start', Bool, start_callback)

# initialize node
rospy.init_node('talker', anonymous=True)

# wait until start is flipped
start = True
while(start):
    start = not rospy.wait_for_message('/start', Bool, timeout=None).data
    sleep(0.1)

# Extend the trailer at the beginning
extend_trailer()

sleep(1)
forward(3500, 125)
sleep(1)
right(350, 100)
sleep(1)
backward(500, 100)
sleep(1)
turn_right()
turn_right()
sleep(1)
backward(800, 75)
sleep(1)
right(300, 100)
sleep(1)
backward(400, 50)
sleep(1)
forward(8000, 150) #slightly higher because of wall friction 
sleep(1)
backward(8000, 150) #slightly higher because of wall friction
sleep(1)
left(600, 100)
sleep(1)
backward(400, 50)
sleep(1)

# first forward path
forward(7400, 100)
sleep(1)
backward(7300, 100)
sleep(1)
backward(800, 50)
sleep(1)
left(600, 100)
sleep(1)
backward(400, 50)
sleep(1)

# second forward path
forward(7400, 100)
sleep(1)
backward(7300, 100)
sleep(1)
backward(800, 50)
sleep(1)
left(600, 100)
sleep(1)
backward(400, 50)
sleep(1)

# third forward path
forward(7400, 100)
sleep(1)
backward(7300, 100)
sleep(1)
left(600,100)
sleep(1)
backward(400, 50)
sleep(1)

# fourth forward path
forward(7400, 100)
sleep(1)
backward(7300, 100)
sleep(1)
left(600,100)
sleep(1)
backward(400, 50)
sleep(1)

# fifth forward path
forward(7400, 100)
sleep(1)
backward(7300, 100)
sleep(1)
left(600,100)
sleep(1)
backward(400, 50)
sleep(1)

# sixth forward path
forward(7400, 100)
sleep(1)
backward(7300, 100)
sleep(1)
left(600,100)
sleep(1)
backward(400, 50)
sleep(1)

# seventh forward path
forward(7400, 100)
sleep(1)
backward(7300, 100)
sleep(1)
left(600,100)
sleep(1)
backward(400, 50)
sleep(1)