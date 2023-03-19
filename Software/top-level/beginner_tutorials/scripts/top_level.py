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

import pygame
from time import sleep

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def pub_direction(x, y, z):
    msg.linear.x = x
    msg.linear.y = y
    msg.angular.z = z
    pub.publish(msg)

# initialize things
global msg
msg = Twist()

pub = rospy.Publisher('/locomotion/cmd_vel', Twist, queue_size=10)
rospy.init_node('talker', anonymous=True)

pygame.init()

# create window with size (our image size)
window = pygame.display.set_mode((700,400))  # track 1
#window = pygame.display.set_mode((1155,399))   # track 2

# load image file
bg  = pygame.image.load("images/track1.png")
#bg  = pygame.image.load("track2.png")
car = pygame.image.load("images/car.png")
car = pygame.transform.scale(car, (40, 40)) # resize car image

""" main loop varibales and things """
# set up timer clock 
clock = pygame.time.Clock()

# initial x y axis position of the car
car_x = 30   
car_y = 260  

JUMP_VALUE = 25     # turning point value
direction = 'y_up'  # cars current direction
run = 1

# start the robot
pub_direction(0,50,0)
DELAY = 2.4
# main loop
while run:
    clock.tick(30)         # update the window/run loop by this speed
    #check for events
    for event in pygame.event.get():
        # quit button clicked
        if event.type == pygame.QUIT:
            run = 0

    # position images
    window.blit(bg, (0, 0))          # load the track image
    window.blit(car, (car_x, car_y)) # the car image

    # record last x, y pos of car
    last_x, last_y = car_x, car_y
    
    # find the center of the car and draw a point on that
    center_x, center_y = (int(car_x + 40 /2), int(car_y + 40 / 2))
    pygame.draw.circle(window, (0,255,255), (center_x, center_y), 5, 5)

    # check surrounding (4 direction data)
    # the calibration value is the pixel from car's sensor/mid point
    # so it checks for road info 30 pixels far from the sensor.
    # 255 means we have a clear white road
    cal_value = 30              # calibrate this to get good data
    y_up      = window.get_at((center_x, center_y - cal_value))[0]
    y_down    = window.get_at((center_x, center_y + cal_value))[0]
    x_right   = window.get_at((center_x + cal_value, center_y))[0]
    x_left    = window.get_at((center_x - cal_value, center_y))[0]
    #print("y_up   ", y_up)
    #print("y_down ", y_down)
    #print("x_right", x_right)
    #print("x_left ", x_left)
    #print("-----------")

    # determine which way to go
    # go up
    if y_up == 255 and direction == 'y_up' and x_left != 255 and x_right != 255:
        # move up
        car_y -= 2  # decrease pixel and move the car on y axis
        
    # make the turn
    if y_up == 255 and direction == 'y_up' and x_left != 255 and x_right == 255:
        # make a right turn
        direction = 'x_right'
        car_y -= JUMP_VALUE
        car_x += JUMP_VALUE
        car = pygame.transform.rotate(car, -90)
        window.blit(car, (car_x, car_y))
        print('Turn Right')
        pub_direction(0,0,50)
        sleep(DELAY)
        pub_direction(0,50,0)

    # go x right
    if y_up != 255 and direction == 'x_right' and y_down != 255 and x_right == 255:
        car_x += 2

    if y_down == 255 and direction == 'x_right' and x_left == 255 and x_right == 255:
        # make a turn from x_right
        car = pygame.transform.rotate(car, -90)
        direction = 'y_down'
        car_y += JUMP_VALUE + 5
        car_x += JUMP_VALUE
        window.blit(car, (car_x, car_y))
        print('Turn Right')
        pub_direction(0,0,50)
        sleep(DELAY)
        pub_direction(0,50,0)

    # go y down
    if y_down == 255 and direction == 'y_down' and x_left != 255 and x_right != 255:
        # move down
        car_y += 2

    # left turn
    if y_down == 255 and direction == 'y_down' and x_left != 255 and x_right == 255:
        # turn from y_down
        car = pygame.transform.rotate(car, 90)
        direction = 'x_right'
        car_y += JUMP_VALUE
        car_x += JUMP_VALUE
        print('Turn left')
        pub_direction(0,0,-50)
        sleep(DELAY)
        pub_direction(0,50,0)
    
    # turn to y up
    if y_up == 255 and direction == 'x_right' and x_left == 255 and x_right == 255:
        # turn from y_down
        car = pygame.transform.rotate(car, 90)
        direction = 'y_up'
        car_y -= JUMP_VALUE + 5
        car_x += JUMP_VALUE
        print('Turn left')
        pub_direction(0,0,-50)
        sleep(DELAY)
        pub_direction(0,50,0)
    
    # if car is stopped
    if car_x == last_x and car_y == last_y:
        # stop the engine sound
        print("STOPPED")
        pub_direction(0,0,0)
        
    pygame.display.update()  # update the window

pygame.quit()      #close everything