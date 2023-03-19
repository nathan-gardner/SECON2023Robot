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

#import rospy
#from std_msgs.msg import String
#from geometry_msgs.msg import Twist

class toplevel:    
    def __init__(self):
        # initialize pygame object
        pygame.init()
        
        # pygame initialized parameters
        self.__window = pygame.display.set_mode((700,400))
        self.__bg = pygame.image.load("images/track1.png")
        self.__car = pygame.image.load("images/car.png")
        self.__car = pygame.transform.scale(self.car, (40, 40)) # resize car image
        self.__clock = pygame.time.Clock()
        self.__car_x = 30   
        self.__car_y = 260  
    
        self.__JUMP_VALUE = 25
        self.__direction = 'y_up'
        self.__run = True
        
    @property
    def window(self):
        return self.__window
    
    @property
    def bg(self):
        return self.__bg
    
    @property
    def car(self):
        return self.__car
    
    @property
    def clock(self):
        return self.__clock
    
    @property
    def car_x(self):
        return self.__car_x
    
    @property
    def car_y(self):
        return self.__car_y
    
    @property
    def JUMP_VALUE(self):
        return self.__JUMP_VALUE
    
    @property
    def direction(self):
        return self.__direction
    
    @property
    def run(self):
        return self.__run
    
    
    def loop(self):
        pass
    
     
    
if __name__ == '__main__':
    try:
        tl = toplevel()
        
    except rospy.ROSInterruptException:
        pass

