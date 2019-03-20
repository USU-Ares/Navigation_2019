#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <wiringPi.h>
#include <iostream>

using namespace std;

//define toggle switche and LED light pins on raspberry pi 3B+ GPIO
#define TOGGLE	     4 	// Toggle ~ toggle switch to switch between manipulation and drive control
#define MAN_LED      5  // LED light to signal Manipulation mode
#define TEL_LED      6  // LED light to signal Teleoperation
#define AUTO_LED     7  // LED light to signal Autonomous mode


class Subsc
{ public:
  
  void drive_Button();
  uint8_t button_x_tel;
  

};

Subsc::drive_Button(const sensor_msgs::Joy::ConstPtr& joy){
 
 Subsc s;
 s.button_x_tel = joy -> buttons[2];

}

void serial() {
 

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
     The "Joy" Ros topic is read in, if Button A is pushed then the Rover is in autonomous mode until the button is pushed again or the toggle is moved to the "on" position */

  else if (Drive_Man == 1)          
   {
      ros::Subscriber joy_in = n.subscribe("joy", 1, s.drive_Button);
      
      if (s.button_x_tel == 0) {
       
         

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
