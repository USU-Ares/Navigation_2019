#include <ros/ros.h>		//Always include for a ROS node
#include <sensor_msgs/Joy.h>	//Include for control messages from game controller
#include <std_msgs/UInt8MultiArray.h>     // To be able to publish uint8 values, need this header file [there is one for mulit array "UInt8MultiArray.h"]
#include <std_msgs/MultiArrayDimension.h> // Where the name for the array being sent is assigned and the size of the array, feeds into UInt8MultiArray.h
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;
// Output Indices 
#define LEFT_JOY 	0 // The left joystick value will be assigned in the first position in the joystick output array.
#define RIGHT_JOY	1 // The right joystick value

// Global Variable
std_msgs::UInt8MultiArray joyoutput;
std_msgs::MultiArrayDimension joyoutput_specs;
unsigned char message_sending[6];          //The array data for the joysticks will be saved in this array and then saved to "data" in joyoutput.

void joystickSaver(const sensor_msgs::Joy::ConstPtr& joy){
  float left_joy = joy -> axes[1]; 
  float right_joy = joy -> axes[4];
  unsigned char l_j;
  unsigned char r_j;
  
/* When the joystick is pulled backward for reverse, the saved joystick values will range from 1 to 127.
   if the joystick is pushed forward the values will range from 128 to 255. When joysticks are at rest 
   a value of 0 will be saved.
*/
  if (left_joy < 0){
    l_j = 128 - 127*abs(left_joy);
  }
  else if (left_joy > 0){
    l_j = 127 + 127*abs(left_joy);
    l_j = l_j + 1;
  } 
  else {
    l_j = 0;
  }

 if (right_joy < 0){
    r_j = 128 - 127*abs(right_joy);
  }
  else if (right_joy > 0){
    r_j = 127 + 127*abs(right_joy);
    r_j = r_j + 1;
  } 
  else {
    r_j = 0;
  }

  message_sending[LEFT_JOY] = l_j;
  message_sending[RIGHT_JOY] = r_j;

  for (int i=2;i<6;i++){
  message_sending[i] = 0;
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
      joyoutput_specs.label = "nav_msg";    // array label name that is given to UInt8MultiArray
      joyoutput_specs.size = 6;             // Lets UInt8MultiArray know how many members are going into its data array.
      joyoutput.layout.dim.clear();
      joyoutput.layout.dim.push_back(joyoutput_specs);

      joyoutput.data.clear();

/* "push_back" only puts one value at a time into the data array in "UInt8MultiArray. The first term entered will be the last term in the array.[3] */
      for (int h=0;h<6;h++){
        joyoutput.data.push_back(message_sending[h]);
      }

      joy_publish.publish(joyoutput);
   
      ros::spinOnce();

      rate.sleep();
   }
 return 0;
}

/*
[1]  UInt8MultiArray [https://answers.ros.org/question/37185/how-to-initialize-a-uint8multiarray-message/]

[2] Example code of working with UInt8MultiArray [https://raw.githubusercontent.com/durovsky/siemens_tutorials/master/siemens_cp1616_io_device_tutorial/doc/doc_siemens_cp1616_io_device_tutorial_node.cpp]

[3]http://docs.ros.org/electric/api/std_msgs/html/UInt8MultiArray_8h_source.html
*/
