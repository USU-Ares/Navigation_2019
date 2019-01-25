#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

std::string sFrame_id = "ins_link";	//default frame id
std::string sChild_frame_id = "camera_link";	//default child frame id

int main(int argc, char** argv){
  ros::init(argc, argv, "test_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(5);
  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
      if(n.param<std::string>("child_frame_id", sChild_frame_id, "camera_link"))
      {
          std::cout << "got the parameter";
      }
      else std::cout << "failed";
    //if(n.hasParam("frame_id")) n.getParam("frame_id", sFrame_id);
    //if(n.hasParam("child_frame_id")) {
    //   std::cout << "Getting child_frame_id";
    //   n.getParam("child_frame_id", sChild_frame_id);
    //}
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.27)), //for sonar sensor tf::Vector3(0.1, 0.0, 0.0)),
        ros::Time::now(),sFrame_id, sChild_frame_id));
	r.sleep();
/*
    broadcaster.sendTransform(
      tf::StampedTransform(
	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
	ros::Time::now(), "odom", "base_link"));
    r.sleep();
*/
/*
    broadcaster.sendTransform(		//I think the nav stack is supposed to calculate and publish the map transform from base link, but we will try this for now
      tf::StampedTransform(
	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
	ros::Time::now(),"map","base_link"));
    r.sleep();
*/
  }
}
