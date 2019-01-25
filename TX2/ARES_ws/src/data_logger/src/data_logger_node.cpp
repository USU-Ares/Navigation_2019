#include <ros/ros.h>                    //The main ROS header
#include <sensor_msgs/PointCloud2.h>    //The message type being subscribed to
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <iostream>                     //for console input and output
#include <fstream>                      //for file input and output
#include <exception>                    //For catching a file open exception
#include <math.h>

//#define _USE_MATH_DEFINES             //needed on older platforms for math.h

/*
 * Author: Derek Workman
 * email: derek.workman@aggiemail.usu.edu
 *
 * This program records /sensor_msgs/PointCloud2 data into a file.
 * The data stored is meant to be taken and measured for standard
 * deviation. This helps provide sensor reading accuracy information.
 *
 * Data: Dec 17, 2018
*/



std::string fileName;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    ros::Time watchdog = ros::Time::now();
    std::ofstream fout;
    fout.open(fileName.c_str(), std::ios::app);
    if(fout.is_open())
    {
        std::cout << "PointCloud2 Data is: " << (msg->is_bigendian ? "bigEndian" : "littleEndian") << '\n';
        //sensor_msgs::PointField::INT8
        sensor_msgs::PointCloud newCloud;       //A regular pointClod is easier to manipulate
        sensor_msgs::convertPointCloud2ToPointCloud(*msg, newCloud);

        std::cout << "Number of points in array: " << newCloud.points.size() << '\n';
        std::cout << "Width x height of input data: " << msg->width*msg->height << '\n';

        //Data is given as one long array of points, and correspond, row after row of camera depth feed
        //Therefore, to get a point at column 3 row 4, the array should be address as PointCloud::points[row*msg->width+column]
        //For 0 <= row <= msg->height-1 and 0 <= column <= msg->width-1


        //For purposes of collecting camera accuracy data, a 5X5 set of depth pixels are selected evenly across the camera input rectangle
        //x, y, and z data points are recorded alone, then also recorded together as sqrt(x^2+y^2+z^2)

        //Create the data header
        fout << "SET 1;;;;;";

        uint32_t N = msg->height-1;     //end values in for-loop are these
        uint32_t M = msg->width-1;

        for (uint32_t row = 0; row < msg->height; row = row + msg->height/4)

        for(uint32_t i = 0; i < newCloud.points.size(); ++i)
        {
            //std::cout << "i = " << i << '\n';

            float dist = sqrt(pow(newCloud.points[i].x,2)+pow(newCloud.points[i].y,2)+pow(newCloud.points[i].z,2));
            if(dist != 0)
            {
                fout << dist;
                if(i < newCloud.points.size() - 1) fout << ", ";
            }
            //fout << newCloud.points[i].z;
            if ((ros::Time::now() - watchdog).toSec() > 10)
            {
                ROS_ERROR("File writing process is taking longer than 10 seconds.\nWriting process aborted\n");
                break;
            }
        }
        fout << ";\n ";
        /*
        for(uint16_t i = 0; i < msg->height; i = i + msg->row_step)
        {
            fout << msg->data[i];
            fout << ';';
            if ((ros::Time::now() - watchdog).toSec() > 10)
            {
                ROS_ERROR("File writing process is taking longer than 10 seconds.\nWriting process aborted\n");
                break;
            }
        }
        */
        std::cout << "The file has been written to\n";
    } else std::cout << "The file could not be opened.\n";

    fout.close();
}

int main(int argc, char **argv)
{
    /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
   ros::init(argc, argv, "pointCloud_dataLogger");


   //store the log file name in RAM
   //std::cout << "fileName is argument number " << argc << '\n';
    if(argc < 2)
    {
        fileName = "dataLog";
    } else fileName = argv[1];   //The file name will be the last input argument

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle n;

    /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
    ros::Subscriber sub = n.subscribe("cloud_in", 1, callback);

    /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
    ros::spin();

    return 0;
}