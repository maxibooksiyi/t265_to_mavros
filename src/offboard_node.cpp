/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;

double position[3]={0,0,0};

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            position[0] = msg->pose.position.x;
            position[1] = msg->pose.position.y;
            position[2] = msg->pose.position.z;
        }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");
    float HEIGHT;
    float SIDE;
    bool AUTO_ARM_OFFBOARD;
    nh1.param<float>("height", HEIGHT, 0.65);
    nh1.param<float>("side", SIDE, 1.5);
    nh1.param<bool>("auto_arm_offboard", AUTO_ARM_OFFBOARD, false);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = HEIGHT;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    int offboard_flag = 0;//防止切land降落之后又去解锁切offboard
    int land_flag = 0;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if(AUTO_ARM_OFFBOARD==true)
        {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))&&(offboard_flag==0)){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))&&(offboard_flag==0)){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();//可以通过这个方式保持当前指点持续几秒钟。
            }
        }
        }
        if(AUTO_ARM_OFFBOARD==false)
        {
        }


        //ros::Duration(1.0).sleep();
        if((abs(position[0])<0.2&&abs(position[1])<0.2&&abs(position[2]-HEIGHT)<0.2)&&(ros::Time::now() - last_request > ros::Duration(5.0))&&(land_flag==0))
        {
         pose.pose.position.x = SIDE;
         pose.pose.position.y = 0;
         pose.pose.position.z = HEIGHT; 
         last_request = ros::Time::now();
        }
        //last_request = ros::Time::now();
        //ros::Duration(1.0).sleep();
        if((abs(position[0]-SIDE)<0.2&&abs(position[1])<0.2&&abs(position[2]-HEIGHT)<0.2)&&(ros::Time::now() - last_request > ros::Duration(5.0)))
        {
         pose.pose.position.x = SIDE;
         pose.pose.position.y = SIDE;
         pose.pose.position.z = HEIGHT; 
         last_request = ros::Time::now();
        }
        if((abs(position[0]-SIDE)<0.2&&abs(position[1]-SIDE)<0.2&&abs(position[2]-HEIGHT)<0.2)&&(ros::Time::now() - last_request > ros::Duration(5.0)))
        {
         pose.pose.position.x = 0;
         pose.pose.position.y = SIDE;
         pose.pose.position.z = HEIGHT; 
         last_request = ros::Time::now();
        }
        if((abs(position[0])<0.2&&abs(position[1]-SIDE)<0.2&&abs(position[2]-HEIGHT)<0.2)&&(ros::Time::now() - last_request > ros::Duration(5.0)))
        {
         pose.pose.position.x = 0;
         pose.pose.position.y = 0;
         pose.pose.position.z = HEIGHT; 
         last_request = ros::Time::now();
         land_flag = 1;
        }
        if((abs(position[0])<0.2&&abs(position[1])<0.2&&abs(position[2]-HEIGHT)<0.2)&&(ros::Time::now() - last_request > ros::Duration(5.0)))
        {
         //pose.pose.position.x = 2;
         //pose.pose.position.y = 2;
         //pose.pose.position.z = 2; 
         set_mode_client.call(land_set_mode);
         if(land_set_mode.response.mode_sent)
           {
             ROS_INFO("land enabled");
             offboard_flag = 1;    
           }  
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



