#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <wiringPi.h>
#include <iostream>
#include <std_msgs/UInt8MultiArray.h> 

using namespace std;

//define toggle switche and LED light pins on raspberry pi 3B+ GPIO
#define TOGGLE	     4 	// Toggle ~ toggle switch to switch between manipulation and drive control
#define MAN_LED      5  // LED light to signal Manipulation mode
#define TEL_LED      6  // LED light to signal Teleoperation
#define AUTO_LED     7  // LED light to signal Autonomous mode

//define joy controller joystick and buttons indices
#define X_BUTTON     2
   


class Subsc
{ public:
  
  void drive_Button();
  uint8_t button_x_tel;
  void NavMsg();
  uint8_t Nav_msg[6];   //Stored array values from nav message to go to "serial()".
  uint8_t Man_msg[6];
  

};



void Subsc::drive_Button(const sensor_msgs::Joy::ConstPtr& joy){
 
 this -> button_x_tel = joy -> buttons[X_BUTTON];

}



void Subsc::NavMsg(const sensor_msgs::UInt8MultiArray::ConstPtr& msg_1){

 


}




void serial(uint8_t message[6],uint8_t drive, uint8_t auto_teleops) {
 Subsc ss;
 uint8_t serial_array[14];
 uint8_t pid_1 = 'C';
 uint8_t pid_2 = 'G';
 uint8_t pid_3 = 0;
 uint8_t Data_Array_Size = 8;  // Number of array members that makes up the data being transmitted. 

 uint8_t crc = 8*9;
  		
/* crc is the ending value for the serial array to let the controller know the max amount of data bytes that are coming in. The 8 represents how many bits there are in a byte and the 9 is the number of byte data members in the serial array. For every 255 bits of data another byte has to be added to crc.*/

 crc = crc/255;
 crc = crc + 1;

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
  
 serial_array[13] = crc; 
  
 }
 

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
  Drive_Man = digitalRead8(TOGGLE)   // Ref [2]
  
  /* If Toggle switch is off then Manipulation msgs will be serialized   and sent to pic controller */
  if (Drive_Man == 0)                
   {
     
   
   }

  /* If Toggle switch is "on" then the rover can be 
     1) Teleoperated
     2) Autonomously driven
     The "Joy" Ros topic is read in, if Button X is pushed then the Rover is in autonomous mode until the button is pushed again or the toggle is moved to the "on" position */

  else if (Drive_Man == 1)          
   {
      ros::Subscriber joy_in = n.subscribe("joy", 1, s.drive_Button);
      
      if (s.button_x_tel == 0) {
        ros::Subscriber Nav_msg = n.subscribe("joy_nav_values",1,
        
        ros::Rate rate(20)
        while (ros::ok()) {
           
           serial(s.Nav_msg, Drive_Man, joy_button value);

           ros::spinOnce();
           rate.sleep();
   
        }
         

      }

      else if (s.button_x_tel == 1) {
      
	 
      }

   }

 }




 return 0;
}


/* References 
[1] https://www.digikey.com/en/maker/blogs/2019/how-to-use-gpio-on-the-raspberry-pi-with-c

[2] https://github.com/WiringPi/WiringPi/blob/master/wiringPi/wiringPi.h




*/
