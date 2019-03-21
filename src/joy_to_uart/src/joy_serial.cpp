#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <wiringPi.h>
#include <iostream>
#include <std_msgs/UInt8MultiArray.h> 
#include <math.h>
#include <serial/serial.h>

using namespace std;

//define toggle switche and LED light pins on raspberry pi 3B+ GPIO
#define TOGGLE_DRIVE_MAN    0 	// Toggle ~ toggle switch to switch between manipulation and drive control
#define TOGGLE_AUTO_TEL    2
#define MAN_LED      3  // LED light to signal Manipulation mode
#define TEL_LED      4  // LED light to signal Teleoperation
#define AUTO_LED     5  // LED light to signal Autonomous mode


   


class Subsc
{ public:
  
  void NavMsg();
  void ManMsg():
  uint8_t Nav_msg[6];   //Stored array values from nav message to go to "serial()".
  uint8_t Man_msg[6];
  

}



/*void Subsc::drive_Button(const sensor_msgs::Joy::ConstPtr& joy){
 
 this -> button_x_tel = joy -> buttons[X_BUTTON];

} */



void Subsc::NavMsg(const sensor_msgs::UInt8MultiArray::ConstPtr& msg_1){
 this->Nav_msg = msg_1->data;      // [3]
}

void Subsc::ManMsg(const sensor_msgs::UInt8MultiArray::ConstPtr& msg_2){
 this->Man_msg = msg_2->data;
}




void serial(uint8_t message[6],uint8_t drive, uint8_t auto_teleops) {
 Subsc ss;
 uint8_t serial_array[13];
 uint8_t pid_1 = 'G';
 uint8_t pid_2 = 'O';
 uint8_t pid_3 = 0;
 uint8_t Data_Array_Size = 8;  // Number of array members that makes up the data being transmitted. 


 serial_array[0] = pid_1;
 serial_array[1] = pid_2;
 serial_array[2] = pid_3;
 serial_array[3] = Serial_Array_Size;

 int j = 4;
 
 for (int i = 0; i < 6; i++) {

    if (i == 0){   
      serial_array[j] = drive;
      j++;   
    }
 
    else if (i == 1){
      serial_array[j] = auto_teleops;
    }
    serial_array[j] = message[i-2];
  }
  
 uint8_t crc = 0;

 for (k=3;k<12;k++){
   for (m=0;m<8;m++){
     crc += (serial_array[k]>>m)&0x01;  //Checks all the data values in the serial_array for 1 bits using "shift"
   }
  }
 
 serial_array[12] = crc;
  
 }
 


int main (){

 // Call in Toggle value from GPIO pin
 wiringPISetup();        // Ref [1]
 pinMode(TOGGLE, INPUT);
 uint8_t Drive_Man;

 ros::init(argc, argv, "joy_serial");
 ros::NodeHandle n;

 Subsc s;

 for (;;) 
 {
  Drive_Man = digitalRead8(TOGGLE_DRIVE_MAN);   // Ref [2]
  Auto_Tel = digitalRead8(TOGGLE_AUTO_TEL);
  
  if (Auto_Tel==1){
      digitalRead8(LED_AUTO,HIGH);
      digitalRead8(LED_TEL,LOW);
  }
  
  else {
      digitalRead8(LED_AUTO,LOW);
      digitalRead8(LED_TEL,HIGH);
  }
  
  /* If Toggle switch is off then Manipulation msgs will be serialized   and sent to pic controller */
  if (Drive_Man == 0)                
   {
      digitalWrite(MAN_LED,LOW);
      
      ros::Subsciber Man = n.subscribe("man_values",1,s.ManMsg);

      ros::Rate rate(20)
      while (ros::ok()) {
           
           serial(s.Man_msg, Drive_Man, Auto_Tel);

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
        ros::Subscriber Nav = n.subscribe("joy_nav_values",1,s.NavMsg);
        
        ros::Rate rate(20)
        while (ros::ok()) {
           
           serial(s.Nav_msg, Drive_Man, Auto_Tel);

           ros::spinOnce();
           rate.sleep();
   
        }
      }

      else if (Auto_Tel == 1) {
      
	 uint8_t auto_msg = [0,0,0,0,0,0];
         
         while (ros::ok()) {
           
           serial(auto_msg, Drive_Man, Auto_Tel);

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




*/
