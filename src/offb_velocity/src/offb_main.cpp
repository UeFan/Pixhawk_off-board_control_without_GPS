/**
 * @file offb_main.cpp
 * @author Julian Oes <julian@oes.ch>
 * @license BSD 3-clause
 *
 * @brief ROS node to do offboard control of PX4 through MAVROS.
 *
 * Initial code taken from http://dev.px4.io/ros-mavros-offboard.html
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>


mavros_msgs::State current_state;


int timeToSendNextVelocity, original_z;
geometry_msgs::PoseStamped local_pose;
geometry_msgs::TwistStamped local_vel;

geometry_msgs::PoseStamped pose;
geometry_msgs::Twist vel;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}


void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_vel = *msg;
}

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose = *msg;
}

void printCurrentMotion()
{
  ROS_INFO("\nTarget_val: %.1f, %.1f, %.1f.",vel.linear.x, vel.linear.y, vel.linear.z);
  ROS_INFO("Current: [%.1f], [%.1f], [%.1f]",local_vel.twist.linear.y, local_vel.twist.linear.y, local_pose.pose.position.z - original_z);
}

void updateControlCommandStats()
{

  std::string connectionStatus = "";
  if (current_state.connected)
    connectionStatus += "[Connected] ";
  else
    connectionStatus += "[Disconnected] ";
  ROS_INFO_STREAM(connectionStatus);
}

/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */
char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}


void sendCommand()
{
  char key = getch();
  switch (key)
  {
    case 65:
    {
      ROS_WARN_STREAM("\nup");
      vel.linear.z += 0.1;
      //printCurrentMotion();
      break;
    }
    case 66:
    {
      ROS_WARN_STREAM("\ndown");
      vel.linear.z -= 0.1;
      //printCurrentMotion();
      break;
    }
    case 'a':
    {
      ROS_WARN_STREAM("\nleft");
      vel.linear.y += 0.1;
      //printCurrentMotion();
      break;
    }
    case 'd':
    {
      ROS_WARN_STREAM("\nright");
      vel.linear.y -= 0.1;
      //printCurrentMotion();
      break;
    }
    case 'w':
    {
      ROS_WARN_STREAM("\nforward");
      vel.linear.x += 0.1;
      //printCurrentMotion();
      break;
    }
    case 's':
    {
      ROS_WARN_STREAM("\nbackward");
      vel.linear.x -= 0.1;
      //printCurrentMotion();
      break;
    }
    default:
    {

    }
  }
}

/*
 * Call main using `rosrun offb offb_main`.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_main");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("mavros/setpoint_position/local", 100);
	// ENU x-East y-North z-Up
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
                                   ("mavros/setpoint_velocity/cmd_vel_unstamped", 100);

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                           ("/mavros/local_position/pose", 100, local_pose_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
                                           //("/mavros/local_position/velocity", 100, local_vel_cb);
					     ("/mavros/global_position/gp_vel", 100, local_vel_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("mavros/set_mode");

    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(20.0);

    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    timeToSendNextVelocity = 20;
    original_z = local_pose.pose.position.z;

     mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int actionOrder = 0;
    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            //if( set_mode_client.call(offb_set_mode) &&
          //          offb_set_mode.response.success) {
                ROS_WARN_STREAM("try disarm......");
          //  }
                updateControlCommandStats();
                last_request = ros::Time::now();
        } else {

            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_WARN_STREAM("Vehicle armed !!!");
                    updateControlCommandStats();
                }
                last_request = ros::Time::now();
            }
        }

        sendCommand();

        if ( actionOrder == 0 )
        {
          if (local_pose.pose.position.z - original_z <= 2)
          {
            vel.linear.z = 0.5;
            local_vel_pub.publish(vel);
          }
          else
          {
            vel.linear.z = 0.0;
            local_vel_pub.publish(vel);
            timeToSendNextVelocity == 0 ? actionOrder++ : timeToSendNextVelocity-- ;
          }
        }

        else if (actionOrder == 1)
        {
	  vel.linear.x = 0.5;
          local_vel_pub.publish(vel);
        }
        printCurrentMotion();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

