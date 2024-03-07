/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>


Eigen::Vector3d pos_drone_t265;
Eigen::Quaterniond q_t265;



void t265_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    //if (msg->header.frame_id == "t265_odom_frame")
    {
        pos_drone_t265 = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        // pos_drone_t265[0] = msg->pose.pose.position.x + pos_offset[0];
        // pos_drone_t265[1] = msg->pose.pose.position.y + pos_offset[1];
        // pos_drone_t265[2] = msg->pose.pose.position.z + pos_offset[2];

        q_t265 = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        //Euler_t265 = quaternion_to_euler(q_gazebo);
        // Euler_t265[2] = Euler_t265[2] + yaw_offset;
        // q_t265 = quaternion_from_rpy(Euler_t265);
    }
    //else
    //{
        //pub_message(message_pub, prometheus_msgs::Message::NORMAL, NODE_NAME, "wrong t265 frame id.");
    //}
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "t265_to_mavros");
    ros::NodeHandle nh;


        //  【订阅】t265估计位置
    ros::Subscriber t265_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 100, t265_cb);

    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);



    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
    



        geometry_msgs::PoseStamped vision;

        vision.pose.position.x = pos_drone_t265[0];
        vision.pose.position.y = pos_drone_t265[1];
        vision.pose.position.z = pos_drone_t265[2];

        vision.pose.orientation.x = q_t265.x();
        vision.pose.orientation.y = q_t265.y();
        vision.pose.orientation.z = q_t265.z();
        vision.pose.orientation.w = q_t265.w();


        vision.header.stamp = ros::Time::now();
        vision_pub.publish(vision);



  

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

