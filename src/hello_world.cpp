/**
 * @file hello_world.cpp
 * @brief Example of c++ file successfully run through ROS
 * @author Amy Phung
 * @author Connor Novak
 */


 #include <iostream>

 main()
 {
     std::cout << "Hello World!";
     return 0;
 }

//
// #include <ros/ros.h>
// #include <std_msgs/String.h>
//
//
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "hell_world");
//     ros::NodeHandle nh;
//
//     ros::Publisher hello_world_pub = nh.advertise<std_msgs::String>
//             ("hello_world", 10);
//
//     //Set ros rate
//     ros::Rate rate(20.0);
//
//     while(ros::ok()){
//
//         hello_world_pub.publish("Hello, World!");
//         ros::spinOnce();
//         rate.sleep();
//     }
//
//     return 0;
// }
