#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <wiringPi.h>
#include <iostream>
#include <std_msgs/UInt8MultiArray.h> 
#include <math.h>
#include <serial/serial.h>
#include <stdint.h>

using namespace std;

//define toggle switches and LED light pins on raspberry pi 3B+ GPIO
#define TOGGLE_DRIVE_MAN    0 	// Toggle switch to switch between manipulation and drive control
#define TOGGLE_AUTO_TEL     2   // Toggle switch to switch between autonomous and teleoperation driving modes
#define MAN_LED      3  // LED light to signal Manipulation mode
#define TEL_LED      4  // LED light to signal Teleoperation
#define AUTO_LED     5  // LED light to signal Autonomous mode
// pin layout rbi 3B+ [https://pi4j.com/1.2/pins/model-3b-plus-rev1.html]


   

/* Subsc contains two functions that subscribe to the ROS topics "joy_nav_values" and "man_values". The values from the ROS topic messages are saved to the serial data . */
class Subsc
{ public:
  
  uint8_t Nav_msg[6];   //Stored array values from navigation's joystick values.
  uint8_t Man_msg[6];   //Stored array values from manipulations joystick values
};



/*void Subsc::drive_Button(const sensor_msgs::Joy::ConstPtr& joy){
 
 this -> button_x_tel = joy -> buttons[X_BUTTON];

} */



void NavMsg(const std_msgs::UInt8MultiArray::ConstPtr& msg_1){
 Subsc s_1;
 for (int ii=0;ii<6;ii++){
   s_1.Nav_msg[ii] = msg_1->data[ii];      // [3]
 }
}

void ManMsg(const std_msgs::UInt8MultiArray::ConstPtr& msg_2){
 Subsc s_2;
 for (int jj=0;jj<6;jj++){
   s_2.Man_msg[jj] = msg_2->data[jj];
 }
}


void serial_fun(uint8_t *message,int num_msg_elem,uint8_t drive, uint8_t auto_teleops) {

 Subsc ss;
 uint8_t serial_array[13];
 uint8_t pid_1 = 'G';
 uint8_t pid_2 = 'O';
 uint8_t pid_3 = 0;
 uint8_t Data_Array_Size = 8;  // Number of array members that makes up the data being transmitted. 

 serial_array[0] = pid_1;
 serial_array[1] = pid_2;
 serial_array[2] = pid_3;
 serial_array[3] = Data_Array_Size;
 serial_array[4] = drive;
 serial_array[5] = auto_teleops;

 int j = 6;
 for (uint8_t i = 0; i < 6; i++) {
    serial_array[j] = *message;  //[4]
    j++;
    message++;
  }

 uint8_t crc = 0;

 for (int k=3;k<12;k++){
   for (int m=0;m<8;m++){
     crc += (serial_array[k]>>m)&0x01;  //Checks all the data values in the serial_array for 1 bits using "shift"
   }
  }
 
 serial_array[12] = crc;
  
 }
 


int main (int argc, char **argv){

 // Call in Toggle value from GPIO pin
 wiringPiSetup();        // Ref [1]
 pinMode(TOGGLE_DRIVE_MAN, INPUT);
 pinMode(TOGGLE_AUTO_TEL, INPUT);
 pinMode(MAN_LED, OUTPUT);
 pinMode(AUTO_LED, OUTPUT);
 pinMode(TEL_LED, OUTPUT);

 uint8_t Drive_Man;
 uint8_t Auto_Tel;
 int num_msg_elem = 6;

 ros::init(argc, argv, "joy_serial");
 ros::NodeHandle n;

 Subsc s;

 ros::Rate rate(20);

 for (;;) 
 {
  Drive_Man = digitalRead(TOGGLE_DRIVE_MAN);   // Ref [2]
  Auto_Tel = digitalRead(TOGGLE_AUTO_TEL);
  
  if (Auto_Tel==1){
      digitalWrite(AUTO_LED,HIGH);
      digitalWrite(TEL_LED,LOW);
  }
  
  else {
      digitalWrite(AUTO_LED,LOW);
      digitalWrite(TEL_LED,HIGH);
  }
  
  /* If Toggle switch is off then Manipulation msgs will be serialized   and sent to pic controller */
  if (Drive_Man == 0)                
   {
      digitalWrite(MAN_LED,LOW);
      
      ros::Subscriber Man = n.subscribe("man_values",1,ManMsg);

      while (ros::ok()) {
           
           serial_fun(s.Man_msg, num_msg_elem, Drive_Man, Auto_Tel);

           ros::spinOnce();
           rate.sleep();
      }
   
   }

  /* If Toggle switch is "on" then the rover can be 
     1) Teleoperated
     2) Autonomously driven
     The "Joy" Ros topic is read in, if Button X is pushed then the Rover is in autonomous mode until the button is pushed again or the toggle is moved to the "on" position */

  else if (Drive_Man == 1)          
   {
      digitalWrite(MAN_LED,HIGH);

      if ( Auto_Tel == 0) {
        ros::Subscriber Nav = n.subscribe("joy_nav_values",1,NavMsg);
        
        ros::Rate rate(20);
        while (ros::ok()) {
           
           serial_fun(s.Nav_msg, num_msg_elem, Drive_Man, Auto_Tel);

           ros::spinOnce();
           rate.sleep();
   
        }
      }

      else if (Auto_Tel == 1) {
      
	 uint8_t auto_msg[6] = {0,0,0,0,0,0};
         
         while (ros::ok()) {
           
           serial_fun(auto_msg, num_msg_elem, Drive_Man, Auto_Tel);

           ros::spinOnce();
           rate.sleep();
   
        }
      }
   }
 }




 return 0;
}


/* References 
[1] https://www.digikey.com/en/maker/blogs/2019/how-to-use-gpio-on-the-raspberry-pi-with-c

[2] https://github.com/WiringPi/WiringPi/blob/master/wiringPi/wiringPi.h

[3] https://raw.githubusercontent.com/durovsky/siemens_tutorials/master/siemens_cp1616_io_device_tutorial/doc/doc_siemens_cp1616_io_device_tutorial_node.cpp

[4] Passing arrays in with pointers [https://beginnersbook.com/2014/01/c-passing-array-to-function-example/]




*/
