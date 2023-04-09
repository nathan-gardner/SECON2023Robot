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

def left_feeding():
    cmd_left_servo_pub_msg.data = "OPEN"
    cmd_left_servo_pub.publish(cmd_left_servo_pub_msg)

def right_feeding():
    cmd_right_servo_pub_msg.data = "OPEN"
    cmd_right_servo_pub.publish(cmd_right_servo_pub_msg)

def extend_trailer():
    cmd_solenoid_pos_msg.data = True
    cmd_solenoid_pos_pub.publish(cmd_solenoid_pos_msg)
    sleep(1)
    cmd_servo_pos_msg.data = "EXTEND"
    cmd_servo_pos_pub.publish(cmd_servo_pos_msg)
    sleep(1)
    cmd_solenoid_pos_msg.data = False
    cmd_solenoid_pos_pub.publish(cmd_solenoid_pos_msg)

def retract_trailer():
    cmd_solenoid_pos_msg.data = True
    cmd_solenoid_pos_pub.publish(cmd_solenoid_pos_msg)
    sleep(1)
    cmd_servo_pos_msg.data = "RETRACT"
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
    while(abs(pos - fr_encoder) < 880):
        pub_direction(0,0,50)
        sleep(0.001)
        fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pub_direction(0,0,0)
    sleep(0.5)

def turn_left():
    pub_direction(0,0,0)
    sleep(0.5)
    global fr_encoder
    fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pos = fr_encoder
    while(abs(pos - fr_encoder) < 880):
        pub_direction(0,0,-50)
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

delay = 1

global cmd_vel_msg
global cmd_servo_pos_msg
global cmd_solenoid_pos_msg
global cmd_left_servo_pub_msg
global cmd_right_servo_pub_msg
cmd_vel_msg = Twist()
cmd_servo_pos_msg = String()
cmd_solenoid_pos_msg = Bool()
cmd_left_servo_pub_msg = String()
cmd_right_servo_pub_msg = String()

# initislize publishers and subscribers 
cmd_vel_pub = rospy.Publisher('/locomotion/cmd_vel', Twist, queue_size=10)
cmd_servo_pos_pub = rospy.Publisher('/duckstorage/cmd_servo_pos', String, queue_size=10)
cmd_solenoid_pos_pub = rospy.Publisher('/duckstorage/cmd_solenoid_pos', Bool, queue_size=10)
cmd_left_servo_pub = rospy.Publisher('/feeding/cmd_left_servo_pos', String, queue_size=10)
cmd_right_servo_pub = rospy.Publisher('/feeding/cmd_right_servo_pos', String, queue_size=10)
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
# extend_trailer()
sleep(delay)
forward(3200, 100)
sleep(delay)
right(850, 125)
sleep(delay)
backward(800, 100)
sleep(delay)
turn_right()
sleep(delay)
backward(600, 75)
sleep(delay)
left(1000, 60)
sleep(delay)
left_feeding()
sleep(delay)
forward(2000, 100)
sleep(delay)
right(300, 75)
sleep(delay)
turn_left()
sleep(delay)
turn_left()
sleep(delay)
backward(200, 50)
sleep(delay)
right(300, 75)
sleep(delay)
right_feeding()
sleep(delay)
forward(200, 75)
sleep(delay)
turn_left()
sleep(delay)
turn_left()
sleep(delay)
backward(1700, 125)
sleep(delay)
right(400, 75)
sleep(delay)
turn_right()
sleep(delay)
right(900, 50)
sleep(delay)
left(200, 60)
sleep(delay)


forward(6100, 100)
sleep(delay)
backward(5800, 100)
sleep(delay)

# second pass
left(900, 125)
sleep(delay)
forward(6100, 100)
sleep(delay)
backward(6000, 100)
sleep(delay)

# third pass
left(900, 125)
sleep(delay)
forward(6100, 100)
sleep(delay)
backward(6000, 100)
sleep(delay)

# fourth pass
left(900, 125)
sleep(delay)
forward(6200, 100)
sleep(delay)

# duck drop
backward(2700, 100)
sleep(delay)
left(500, 50)
sleep(delay)
right(400, 75)
sleep(delay)
# drop trailer
forward(2600, 100)
sleep(delay)
forward(300, 50)
sleep(delay)
right(1200, 125)
sleep(delay)
