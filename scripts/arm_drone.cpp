/**
 * @file hello_world.cpp
 * @brief Sends arming, takeoff, and landing commands to drone using mavros
 based on http://docs.erlerobotics.com/simulation/vehicles/erle_copter/tutorial_3
 * @author Amy Phung
 * @author Connor Novak
 */


/*
USAGE:
-Verify CMakeLists.txt has lines
* add_executable(arm_drone src/arm_drone.cpp)
* target_link_libraries(arm_drone ${catkin_LIBRARIES})
* add_dependencies(arm_drone)
-catkin_make package
-roslaunch mavros px4.launch
-rosservice call /mavros/set_mode 0 "GUIDED"
-rostopic echo /mavros/state to verify GUIDED mode
-rosrun test_package arm_drone
*/

 #include <cstdlib>

 #include <ros/ros.h>
 #include <mavros_msgs/CommandBool.h>
 #include <mavros_msgs/CommandTOL.h>
 #include <mavros_msgs/SetMode.h>

 int main(int argc, char **argv)
 {

     int rate = 10;

     ros::init(argc, argv, "mavros_takeoff");
     ros::NodeHandle n;

     ros::Rate r(rate);


//Mode setting not currently working
//     ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
//     mavros_msgs::SetMode srv_setMode;
//     srv_setMode.request.base_mode = 0;
//     srv_setMode.request.custom_mode = "GUIDED";


     ////////////////////////////////////////////
     ///////////////////ARM//////////////////////
     ////////////////////////////////////////////
     ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
     mavros_msgs::CommandBool srv;
     srv.request.value = true;
     if(arming_cl.call(srv)){
         ROS_ERROR("ARM send ok %d", srv.response.success);
     }else{
         ROS_ERROR("Failed arming or disarming");
     }

     ////////////////////////////////////////////
     /////////////////TAKEOFF////////////////////
     ////////////////////////////////////////////
     ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
     mavros_msgs::CommandTOL srv_takeoff;
     srv_takeoff.request.altitude = 10;
     srv_takeoff.request.latitude = 0;
     srv_takeoff.request.longitude = 0;
     srv_takeoff.request.min_pitch = 0;
     srv_takeoff.request.yaw = 0;
     if(takeoff_cl.call(srv_takeoff)){
         ROS_ERROR("srv_takeoff send ok %d", srv_takeoff.response.success);
     }else{
         ROS_ERROR("Failed Takeoff");
     }

     ////////////////////////////////////////////
     /////////////////DO STUFF///////////////////
     ////////////////////////////////////////////
     sleep(10);

     ////////////////////////////////////////////
     ///////////////////LAND/////////////////////
     ////////////////////////////////////////////
     ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
     mavros_msgs::CommandTOL srv_land;
     srv_land.request.altitude = 10;
     srv_land.request.latitude = 0;
     srv_land.request.longitude = 0;
     srv_land.request.min_pitch = 0;
     srv_land.request.yaw = 0;
     if(land_cl.call(srv_land)){
         ROS_INFO("srv_land send ok %d", srv_land.response.success);
     }else{
         ROS_ERROR("Failed Land");
     }

     while (n.ok())
     {
       ros::spinOnce();
       r.sleep();
     }


     return 0;
 }
