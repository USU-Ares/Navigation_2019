#include <ros/ros.h>		//Always include for a ROS node
#include <sensor_msgs/Joy.h>	//Include for control messages from game controller
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;
// Output Indices 
#define LEFT_JOY 	0 // The left joystick value will be assigned in the first position in the joystick output array.
#define Right_JOY	1
#define DIR		2 //DIR is the third term in the joysick output array that will be published.
#define BUTTONS         3 

/* Joystick Direction Bit Indices and Buttons Bit Indices */
#define L_DIR	       0x01
#define R_DIR          0x02
#define BUTTON_A       0x01 // A button: Soft Kill Stop for Rover
#define BUTTON_X       0x02 // X button: switch between teleops and 
                            //  autonomous mode.

/* joystickSaver() returns two vectors: one that has the y component of the joystick inputs and one for the x and square button pushed.
  The construct sensor_msgs::Joy::ConstPtr& joy creates the Joy.h   object inside the code and saves it as joy.
  The apperstand "&" means passing by reference, the variable can be changed at its storage location between codes. */

// Global Variable
volatile uint8_t joy_output[4];

void joystickSaver(const sensor_msgs::Joy::ConstPtr& joy){
  int left_joy = joy -> axis[1];
  int right_joy = joy -> axis[4];
  int button_a_stop = joy -> buttons[0];
  int button_x_tel = joy -> buttons[2];
  left_joy = left_joy*255;
  right_joy = right_joy*255;
  /*
   "joy_output" for its first element gives the joystick value from   0 to 255, the second element is the righ joystick value, third is a bit array that gives the direction of the wheels, and fourth is the stop and teleoperation on/off buttons. 
*/

  joy_output[LEFT_JOY] = left_joy;
  joy_output[RIGHT_JOY] = right_joy;        

  uint8_t dir [8] = {0};
  uint8_t button_press[8] = {0};

  // If the joystick value is negative (for reverse) a 0 is set, else a 1.
  if (left_joy < 0){
      dir[7] = dir[7]&L_DIR;
   }else if(left_joy >= 0){
      dir[7] = dir[7]|L_DIR;
    }
  if (right_joy < 0){
     dir[6] = dir[6]&R_DIR;
  }else if(right_joy >= 0){
     dir[6] = dir[6]|R_DIR; 
   }

  joy_output[DIR] = dir;
 
  // If a button is pushed a 1 is assigned at the associated bit index, else it is 0.
  if (button_a_stop == 1){
     button_press[7] = button_press[7]|0x01;
  }else {
     button_press[7] = button_press[7]&0x01;
   }

  if (button_x_tel == 1){
     button_press[6] = button_press[6]|0x02;
  } else {
     button_press[6] = button_press[6]&0x02;
   }

  joy_output[BUTTONS] = button_press;

}



int main(int argc, char **argv){
  ros::init(argc, argv, "joy_reciever");
  ros::NodeHandle n;
  // Subscribe to Joy topic to recieve the game controller inputs
  ros::Subscriber joy_in = n.subscribe("joy", 1, joystickSaver);

  ros::Publisher joy_publish = n.advertise("joy_values", 1);

  ros::Rate rate(20);
  while (ros::ok()) {
      joy_publish.publish(joy_output);
   
      ros::spinOnce();

      rate.sleep();
   }

 return 0;
}
