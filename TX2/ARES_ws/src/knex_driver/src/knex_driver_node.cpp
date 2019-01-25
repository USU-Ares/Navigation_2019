/* knex_driver_node:
 * This ROS node drives the k'nex test rover, by taking in joy commands
 * and converting to motor speed/direction outputs at the serial port.
 * An Arduino Uno recieves the serial commands, and drives the motors.
 * 
 * ROS inuput messages:
 * 	/joy (sensor_msgs/Joy.h)
 * ROS output messages:
 * 	none
 * Other inputs:
 * 	none
 * Other outputs:
 * 	serial data to Arduino Uno
 * 	Protocol: UART
 * 	Packet organization: PID|DATA|CRC -> "GO"|direction(b'000000rl'),motorL,motorR|
 * 		direction bits are low for forward, and high for backward
 * 		motorL and motorR are uint8_t data bytes, where 0=stop and 255=full speed
 *
 * Helpful Links:
 * 	https://answers.ros.org/question/12181/how-to-use-sensor_msgsjoy-instead-of-joystick_driversjoy/
 *
 * Author Derek Workman / Seth Horne
 * Email: derek.workman@aggiemail.usu.edu
 */


/*****Include Files*******/
#include <ros/ros.h>		//Always include for a ROS node
#include <sensor_msgs/Joy.h>	//Include for control messages from game controller
#include <serial/serial.h>	//Include for serial output to Arduino motor driver
#include <math.h>		//For abs() and possibly other mathimatical opperations required
#include <string.h>		//for passing the serial port name
#include <iostream>		//for temporary debug

/******Definitions********/
//indecies for serial packet
#define _G	0	//Packet ID 'G'
#define _O	1	//Packet ID 'O'
#define DIR	2	//Direction Byte b'000000lr'
#define MOTOR_L	3	//Left motor speed 0 to 255
#define MOTOR_R 4	//Right motor speed 0 to 255
#define CRC	5	//Cyclic Redundancy Check, contains the sum of all data bits.
			//	this is used for error checking
//Constant Sizes
#define N	6	//Total number of bytes in serial packet
#define N_PID	2	//Number of bytes that make up the PID
#define N_DATA  3	//Number of data bytes
#define N_CRC	1	//Number of CRC bytes

//Bit masking
#define DIR_L_BIT	0x01	//Bit address for left wheel direction
#define DIR_R_BIT	0x02	//Bit address for right wheel direction

//Strings
#define SERIAL_PORT_NAME	"/dev/ttyACM0"	//Consider making this a ROS parameter so that it can be altered

//Indecies for joy message
#define LEFT_STICK	1	//index for Joy message array
#define RIGHT_STICK	4	//indec for Joy message array

//Safety Timeout
#define TIMEOUT		5.000	//Motor timeout after x seconds

/****Global Variables*****/
uint8_t packet[N]	= {0};	//contains the packet to send via serial
serial::Serial ser;	//The serial object
double lastTime = 0;	//for checking joy timeout

/******Main Program*******/

//Function to initialize the serial port
bool InitSerial(uint32_t baud, std::string portName) {
    serial::Timeout timeout = serial::Timeout(0, 0, 0, 0, 0);
    serial::bytesize_t bytesize = serial::eightbits;
    serial::parity_t parity = serial::parity_none;
    serial::stopbits_t stopbits = serial::stopbits_one;
    serial::flowcontrol_t flowcontrol = serial::flowcontrol_none;

    try{
        ser.setPort(portName);
        ser.setBaudrate(baud);
        ser.setTimeout(timeout);
        ser.setBytesize(bytesize);
        ser.setParity(parity);
        ser.setStopbits(stopbits);
        ser.setFlowcontrol(flowcontrol);
        ser.open();
    }
    catch (serial::SerialException e) {
        ROS_FATAL("Failed to connect to the Motor Controller, %s.", e.what());
        ros::shutdown();	//Halt the ROS system when critical components are not functioning
        return false;
    }
    return true;
}

//add data array for CRC
uint8_t GetCRC(uint8_t* buf, size_t n) {

  uint8_t bitCount = 0;
  
  for(uint8_t i = 0; i < n; i++) {
    bitCount += buf[i]&0x01;  //count the first bit if set
    for(uint8_t j = 1; j < 8; j++) {
      bitCount += (buf[i] >> j)&0x01; //count the rest of the set bits
    }
  }
  return bitCount;
}

//ROS callback for joy topic
//	Motor drive data is updated here
void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    lastTime = ros::Time::now().toSec();	//update last time variable
    if(msg->axes[LEFT_STICK] < 0) {
        packet[DIR] = (packet[DIR]&DIR_R_BIT)|DIR_L_BIT;	//Set DIR_L bit if reverse, keep DIR_R as is
    }else packet[DIR] = (packet[DIR]&DIR_R_BIT);	//else clear DIR_L, leave DIR_R
    if(msg->axes[RIGHT_STICK] < 0) {
        packet[DIR] = (packet[DIR]&DIR_L_BIT)|DIR_R_BIT;	//Set DIR_L bit if reverse, keep DIR_L as is
    }else packet[DIR] = (packet[DIR]&DIR_L_BIT);	//else clear the DIR_R bit, leave DIR_L
    packet[MOTOR_L] = abs(((uint8_t)0xff*msg->axes[LEFT_STICK]));
    packet[MOTOR_R] = abs(((uint8_t)0xff*msg->axes[RIGHT_STICK]));

    uint8_t data[N_DATA];	//data length will be N subtract "GO" and CRC byte
    for(uint8_t i = 0; i < (N_DATA); i++) {
        data[i] = packet[i+N_PID];
    }
    packet[CRC] = GetCRC(data, N_DATA);
}

//Main
int main(int argc, char **argv) {

    ros::init(argc,argv,"knex_driver");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joy",100,JoyCallback);

    ros::Rate rate(20);	//repeat transmission at 20Hz

    packet[_G] = 'G';	//initialize packet ID
    packet[_O] = 'O';	//initialize packet ID

    InitSerial(9600, SERIAL_PORT_NAME);	//initialize serial port with 9600 baud rate

    while(ros::ok()) {
        if((ros::Time::now().toSec() - lastTime) > TIMEOUT) {	//if a new packet has not been recieved within the amount of seconds specified by TIMEOUT
            for(uint8_t i = N_PID; i < N; i++) {                //      then stop the motors
                packet[i] = 0;
            }
        }
        ser.write(packet, N);	//Send Packet over serial
        ros::spinOnce();	//Watch for callbacks
        rate.sleep();		//sleep between data transmissions
    }
    ser.close();
    return 0;
}
