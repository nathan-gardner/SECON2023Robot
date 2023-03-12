#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

// Locomotion Defs
// same driver
// Front Left Motor Out34 Yellow -, Brown +
// Front Right Motor Out12 Brown +, Orange -

// same driver
// Rear Left Motor Out12 Yellow -. Black +
// Rear Right Motor Out34 Red +, Black -

#define FRONT_LEFT_PIN1 26
#define FRONT_LEFT_PIN2 28
#define FRONT_RIGHT_PIN1 22
#define FRONT_RIGHT_PIN2 24
#define REAR_LEFT_PIN1 30
#define REAR_LEFT_PIN2 32
#define REAR_RIGHT_PIN1 34
#define REAR_RIGHT_PIN2 36

#define FRONT_LEFT_SPEED_PIN 10
#define FRONT_RIGHT_SPEED_PIN 11
#define REAR_LEFT_SPEED_PIN 12
#define REAR_RIGHT_SPEED_PIN 13

// ENCA always need to be at an interrupt pin
#define FRONT_LEFT_ENCA 19
#define FRONT_LEFT_ENCB 5
#define FRONT_RIGHT_ENCA 2
#define FRONT_RIGHT_ENCB 6
#define REAR_LEFT_ENCA 18
#define REAR_LEFT_ENCB 7
#define REAR_RIGHT_ENCA 3
#define REAR_RIGHT_ENCB 4

#define CLICKS_PER_ROTATION 562.2

namespace locomotion
{
// Locomotion Data
extern geometry_msgs::Twist t_stateMotorLocomotion;
extern std_msgs::UInt32MultiArray u32_motorPosData;
// array format front_left, front_right, rear_left, rear_right
extern uint32_t enc_pos[4];
extern volatile uint32_t enc_pos_i[4];
extern uint32_t enc_posPrev[4];
extern float enc_vel[4];
extern volatile float enc_vel_i[4];
extern float xyz[4];

extern long prevT;
extern float deltaT;

extern std_msgs::Float32MultiArray af32_velocity;
extern ros::Publisher velocity;

// Locomotion Pub/Sub
extern ros::Publisher motorState;
extern ros::Publisher encoder;
extern ros::Subscriber<geometry_msgs::Twist> cmd_vel;

/**
 * @brief Digital write to motor_pin1 and motor_pin2 to command direction and speed. Used for the locomotion subsystem.
 *
 * @param motor_pin1 Pin1 (digital value) for locomotion motor for which the direction is being set
 * @param motor_pin2 Pin2 (digital value) for locomotion motor for which the direction is being set
 * @param speed_pin Speed pin (analog value) for locomotion motor for which the speed is being set
 * @param motor_speed [-1.0,1.0] range for from Twist message which encodes cmd direction and speed
 */
void set_motor_speed(int motor_pin1, int motor_pin2, int speed_pin, float motor_speed);

/**
 * @brief Decodes geometry_msgs::Twist message to get linear x and y and angular z. Calculates each wheel speed and
 * directions and then writes values to pins.
 *
 * @param cmd_vel Updated geometry_msgs::Twist value published to /locomotion/cmd_vel
 */
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel);

void set_locomotion_speed();

/**
 * @brief Update published encoder value
 *
 */
void updateEncoder();

void updateVelocity();

/**
 * @brief interrupt service routine for front left encoder , increments position + or - based on direction
 *
 */
void readFrontLeftEncoder();

/**
 * @brief interrupt service routine for front right encoder , increments position + or - based on direction
 *
 */
void readFrontRightEncoder();

/**
 * @brief interrupt service routine for rear left encoder , increments position + or - based on direction
 *
 */
void readRearLeftEncoder();

/**
 * @brief interrupt service routine for rear right encoder , increments position + or - based on direction
 *
 */
void readRearRightEncoder();

// updates value pointed to by vel
void computeVelocity(volatile float* vel);

// updates value pointed to by vel
void lowPassFilter(volatile float* vel);

// updates value pointed to by pwr
void pi_control(float* vel, int* pwr, float* xyz);

/**
 * @brief Initialization for the locomotion namespace
 *
 * @param nh Pointer to the ROS node handle
 */
void init(ros::NodeHandle* nh);

}  // namespace locomotion

#endif