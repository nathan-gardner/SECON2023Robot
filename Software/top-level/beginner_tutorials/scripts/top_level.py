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
 ******************************************************************************/
'''

from time import sleep

import rospy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

def pub_direction(x, y, z):
    msg.linear.x = x
    msg.linear.y = y
    msg.angular.z = z
    pub.publish(msg)

'''
#broken
def turn_left():
    pub_direction(0,0,0)
    sleep(0.5)
    global fr_encoder
    fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pos = fr_encoder
    while(abs(pos - fr_encoder) > 740):
        pub_direction(0,0,-50)
        sleep(0.1)
        fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pub_direction(0,0,0)
    sleep(0.5)
'''

# working
def turn_right():
    pub_direction(0,0,0)
    sleep(0.5)
    global fr_encoder
    fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pos = fr_encoder
    while(abs(pos - fr_encoder) < 740):
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
        pub_direction(0,50,0)
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

def left(clicks):
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


'''
def left(clicks):
    global fr_encoder
    global rl_encoder
    fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    rl_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[2]
    fr_pos = fr_encoder
    rl_pos = rl_encoder
    while((abs(fr_pos - fr_encoder) < clicks) and (abs(rl_pos - rl_encoder) < clicks)):
        pub_direction(50,0,0)
        sleep(0.001)
        fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
        rl_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[2]
    pub_direction(0,0,0)
'''

def right(clicks):
    global fr_encoder
    fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pos = fr_encoder
    while(abs(pos - fr_encoder) < clicks):
        pub_direction(-50,0,0)
        sleep(0.001)
        fr_encoder = rospy.wait_for_message('/locomotion/encoder', Twist, timeout=None).data[1]
    pub_direction(0,0,0)

def callback(data):
    print(str(data.data[0]) + ' ' + str(data.data[1]) + ' ' + str(data.data[2]) + ' ' + str(data.data[3]))
    fl_encoder = data.data[0]
    fr_encoder = data.data[1]
    rl_encoder = data.data[2]
    rr_encoder = data.data[3]

# initialize things
fl_encoder = 0
fr_encoder = 0
rl_encoder = 0
rr_encoder = 0

global msg
msg = Twist()

pub = rospy.Publisher('/locomotion/cmd_vel', Twist, queue_size=10)
sub = rospy.Subscriber('/locomotion/encoder', Int32MultiArray, callback)
rospy.init_node('talker', anonymous=True)

sleep(1)
forward(3800, 75)
sleep(1)
right(400)
sleep(1)
backward(240, 50)
sleep(1)
turn_right()
turn_right()
sleep(1)
backward(800, 50)
sleep(1)
right(300)
sleep(1)
left(600)
sleep(1)
backward(400, 50)
sleep(1)

# first forward path
forward(6500, 75)
sleep(1)
backward(6500, 100)
sleep(1)
backward(800, 50)
sleep(1)
left(400)
sleep(1)

# second forward path
forward(6500, 75)
sleep(1)
backward(6500, 100)
sleep(1)
backward(800, 50)
sleep(1)
left(400)
sleep(1)

# third forward path
forward(6500, 75)
sleep(1)
backward(6500, 100)
sleep(1)

'''
sleep(1)
forward(3800, 75)
sleep(1)
right(400)
sleep(1)
backward(240, 50)
sleep(1)
turn_right()
sleep(1)
turn_right()
sleep(1)
backward(800, 50)
sleep(1)
right(300)
sleep(1)
left(600)
sleep(1)
backward(400, 50)
sleep(1)
forward(6500, 75)
sleep(1)
backward(6500, 150)
sleep(1)
backward(800, 50)
sleep(1)
left(600)
sleep(1)
forward(6500, 75)
sleep(1)
backward(6500, 150)
sleep(1)
backward(800, 50)
sleep(1)
left(600)
sleep(1)
forward(6500, 75)
sleep(1)
backward(6500, 150)
'''


