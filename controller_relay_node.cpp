#include <ros/ros.h>
#include <std_msgs/String.h>


//[https://answers.ros.org/question/12181/how-to-use-sensor_msgsjoy-instead-of-joystick_driversjoy/]



void joyCallback(const std_msgs::Joy::ConstPtr& msg)
{
   
         R = msg->axes[2];
      
    
         L = msg->axes[4];
   
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Joy");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("Joy",1000,joyCallback);

    ros::spin();
    return 0;
}