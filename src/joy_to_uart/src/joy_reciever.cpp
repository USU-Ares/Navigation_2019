#include <ros/ros.h>		//Always include for a ROS node
#include <sensor_msgs/Joy.h>	//Include for control messages from game controller
#include <std_msgs/UInt8MultiArray.h>     // To be able to publish uint8 values, need this header file [there is one for mulit array "UInt8MultiArray.h"]
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;
// Output Indices 
#define LEFT_JOY 	0 // The left joystick value will be assigned in the first position in the joystick output array.
#define RIGHT_JOY	1
#define DIR_L		2 //DIR is the third term in the joysick output array that will be published.
#define DIR_R           3


/* Joystick Direction Bit Indices and Buttons Bit Indices 
#define L_DIR	       0x01
#define R_DIR          0x02
*/

/* joystickSaver() returns two vectors: one that has the y component of the joystick inputs and one for the x and square button pushed.
  The construct sensor_msgs::Joy::ConstPtr& joy creates the Joy.h   object inside the code and saves it as joy.
  The apperstand "&" means passing by reference, the variable can be changed at its storage location between codes. */

// Global Variable
std_msgs::UInt8MultiArray joyoutput;

void joystickSaver(const sensor_msgs::Joy::ConstPtr& joy){
  uint8_t left_joy = joy -> axes[1]; 
  uint8_t right_joy = joy -> axes[4];
  left_joy = left_joy*255;
  right_joy = right_joy*255;
  joyoutput.data[LEFT_JOY] = left_joy;
  joyoutput.data[RIGHT_JOY] = right_joy;
  /*
   "joyoutput" for its first element gives the joystick value from   0 to 255, the second element is the righ joystick value, third is a bit array that gives the direction of the wheels, and fourth is the stop and teleoperation on/off buttons. 
*/
   

  uint8_t dir_L;
  uint8_t dir_R;

  // If the joystick value is negative (for reverse) a 0 is set, else a 1.

  if (left_joy < 0){
      //dir = dir&L_DIR;
      dir_L = 0;
   }else if(left_joy >= 0){
     // dir = dir|L_DIR;
      dir_L = 1;
    }
  if (right_joy < 0){
     //dir = dir&R_DIR;
      dir_R = 0;
  }else if(right_joy >= 0){
     //dir = dir|R_DIR; 
      dir_R = 1;
   }


  joyoutput.data.[DIR_L] = dir_L;
  joyoutput.data[DIR_R] = dir_R;

  for (int i=4;i<6;i++){
  joyoutput.data[i] = 0;
  }

}



int main(int argc, char **argv){
  ros::init(argc, argv, "joy_reciever");
  ros::NodeHandle n;
  // Subscribe to Joy topic to recieve the game controller inputs
  ros::Subscriber joy_in = n.subscribe("joy", 1, joystickSaver);

  ros::Publisher joy_publish = n.advertise<std_msgs::UInt8MultiArray>("nav_joy_values", 1);

  ros::Rate rate(20);
  while (ros::ok()) {
      joy_publish.publish(joyoutput);
   
      ros::spinOnce();

      rate.sleep();
   }

 return 0;
}

/*
[1] Example how to work with UInt8MultiArray [https://answers.ros.org/question/37185/how-to-initialize-a-uint8multiarray-message/]
*/
