#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
//#include <inertial_sense/inertial_sense.h> //Could not get c_make to find this
#include <nav_msgs/Odometry.h>
#include <iostream>

//look up geometry_msgs/Odometry to finish this subscriber/publisher

tf::Quaternion ins_quat(0, 0, 0, 1);	//Initial values for INS transform frame from world
tf::Vector3 ins_vect(0, 0, 0);

//tf::Quaternion initial_quat;  //only set origin of position, not orientation
tf::Vector3 initial_vect;
bool first_run = true;

void insCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static tf::TransformBroadcaster broadcaster;
    if (first_run) {
        first_run = false;
    /*  Do not want to preset orientation  
        initial_quat = tf::Quaternion(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
   */
        initial_vect = tf::Vector3(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
    }

    ins_quat = tf::Quaternion(
    /*  Only set the position vector to the 0 origin, the same thing should not be done to the orientation
        -(msg->pose.pose.orientation.x - initial_quat.x()),
        -(msg->pose.pose.orientation.y - initial_quat.y()),
        -(msg->pose.pose.orientation.z - initial_quat.z()),
        1-(msg->pose.pose.orientation.w - initial_quat.w())
    );
    */
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    ins_vect = tf::Vector3(
        msg->pose.pose.position.x - initial_vect.x(),
        msg->pose.pose.position.y - initial_vect.y(),
        msg->pose.pose.position.z - initial_vect.z()
    );

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(
          ins_quat, ins_vect
        ),
        msg->header.stamp,//ros::Time::now(),
        "world", "ins_link"
      )
    );

    broadcaster.sendTransform(  //TODO Should make this a static tf in the future
      tf::StampedTransform(
        tf::Transform(
          tf::Quaternion(0, 0, 0.707, 0.707), tf::Vector3(-0.05, -0.165, -0.345) //static orientation $
        ),
        msg->header.stamp,//ros::Time::now(),
        "ins_link", "camera_link"
      )
    );
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle n;

 // ros::Rate r(5);

  ros::Subscriber ins_sub = n.subscribe("ins", 1, insCallback);
/* I chose to move this to the callback in order to update as soon as possible
  while(n.ok()){

    broadcaster.sendTransform(	//TODO Should make this a static tf in the future
      tf::StampedTransform(
        tf::Transform(
	  tf::Quaternion(0, 0, -0.707, 0.707), tf::Vector3(0.165, 0.05, 0.345) //static orientation between uINS and RealSense camera
        ),
        ros::Time::now(),"ins_link", "camera_link")
      );

    broadcaster.sendTransform(
      tf::StampedTransform(
	tf::Transform(
	  ins_quat, ins_vect
	),
        ros::Time::now(),"World", "ins_link")
      );
	r.sleep();
*/
  ros::spin();
}
