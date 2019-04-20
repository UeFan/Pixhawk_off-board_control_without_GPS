/**
 * @file offb_main.cpp
 * @license BSD 3-clause
 *
 * @brief ROS node to do offboard control of PX4 through MAVROS.
 */

#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
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
#include <math.h>
#define COUNT 30

///R = 8*round_speed*COUNT/20*PI
mavros_msgs::State current_state;
int round_flag_r =0;
int round_flag_l =0;
int count=COUNT,count_r=0,count_l=0;
int original_z;
geometry_msgs::PoseStamped local_pose;
geometry_msgs::TwistStamped local_vel;

geometry_msgs::PoseStamped pose;
geometry_msgs::Twist vel, vel_;

// Joystick axis
int joystick_linear_axis_   = 4;
int joystick_angular_axis_  = 3;
int joystick_yaw_axis_      = 0;
int joystick_altitude_axis_ = 1;
float joystick_deadzone_    = 0.1f;
// Joystick buttons
int joystick_dnn_on_button_    = 0;
int joystick_dnn_off_button_   = 1;
int joystick_dnn_left_button_  = 4;
int joystick_dnn_right_button_ = 5;
float angular_control_val_   = 0;  // forward control: "+" is forward and "-" is back (-1..1), updated in the JOYSTICK subsriber
float north_control_val_  = 0; // turn control: "+" turns left and "-" turns right (-1..1), updated in the JOYSTICK subsriber
float east_control_val_      = 0;     // yaw control: "+" rotates in place to left and "-" rotates right (-1..1)
float altitude_control_val_ = 0; // altitude control: "+" is up and "-" is down (-1..1)


float delta_z, delta_x, delta_y,round_speed=0.3;

/*
 For short-range Cartesian representations of geographic locations, use the east north up [5] (ENU) convention:
 
 X east
 Y north
 Z up
 */

//------------8 step-----> 180/9 = 20.
const float delta_angle = 20;
float start_angle = 0;
float current_angle = 0;   // current_angle = theta - start_angle, theta = [0,180]

void turn_left_round()
{
  current_angle = 180 + (count_l+1)*(-delta_angle) - start_angle;
  vel.linear.x = round_speed*cos(current_angle/180*3.1415) + delta_x;
  vel.linear.y = round_speed*sin(current_angle/180*3.1415) + delta_y;
  vel.linear.z = delta_z;
  
  
  count --;
  if(count == 0)
  {
    count = COUNT;
    count_l ++;
  }
  
  if(count_l == 8)
  {
    round_flag_l = 0;
    count_l = 0;
  }
}
void turn_right_round()
{
  current_angle = (count_r+1)*delta_angle - start_angle;
  vel.linear.x = round_speed*cos(current_angle/180*3.1415) + delta_x;
  vel.linear.y = round_speed*sin(current_angle/180*3.1415) + delta_y;
  vel.linear.z = delta_z;
  

  
  count --;
  if(count == 0)
  {
    count = COUNT;
    count_r ++;
  }
  
  if(count_r == 8)
  {
    round_flag_r = 0;
    count_r = 0;
  }
}


void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
//    ROS_INFO_STREAM(current_state.mode);
}

void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_vel = *msg;
}

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose = *msg;
}

void joystick_cb(const sensor_msgs::Joy::ConstPtr& msg)
{
  float mov_stick_updown = msg->axes[3];
  float mov_stick_leftright = msg->axes[2];
  float pos_stick_updown = msg->axes[1];
  float pos_stick_leftright = msg->axes[0];
  ROS_INFO("JOY_VALS:  height=%2.2f, north=%2.2f, east=%2.2f",mov_stick_leftright, pos_stick_leftright, pos_stick_updown);
//,
  //          msg->buttons[joystick_dnn_left_button_], msg->buttons[joystick_dnn_right_button_],
    //        msg->buttons[joystick_dnn_on_button_], msg->buttons[joystick_dnn_off_button_]
  
  angular_control_val_   = fabs(mov_stick_updown)>joystick_deadzone_    ? mov_stick_updown : 0;
  altitude_control_val_  = fabs(mov_stick_leftright)>joystick_deadzone_ ? mov_stick_leftright : 0;
  east_control_val_ = fabs(pos_stick_updown)>joystick_deadzone_    ? pos_stick_updown : 0;
  north_control_val_      = fabs(pos_stick_leftright)>joystick_deadzone_ ? pos_stick_leftright : 0;
  
}

bool setJoystickParams(std::string joy_type)
{
  // Axes values: 1 = left/top, 0 = center, -1 = right/bottom
  boost::algorithm::to_lower(joy_type);
  if (joy_type == "shield" || joy_type == "xbox_wireless")
  {
    joystick_linear_axis_ = 3;
    joystick_angular_axis_ = 2;
    joystick_altitude_axis_ = 1;
    joystick_yaw_axis_ = 0;
  }
  else if (joy_type == "xbox_wired")
  {
    joystick_linear_axis_ = 4;
    joystick_angular_axis_ = 3;
    joystick_altitude_axis_ = 1;
    joystick_yaw_axis_ = 0;
  }
  else if (joy_type == "shield_2017")
  {
    joystick_linear_axis_ = 5;
    joystick_angular_axis_ = 2;
    joystick_altitude_axis_ = 1;
    joystick_yaw_axis_ = 0;
  }
  else
  {
    ROS_FATAL("Unsupported joystick type: %s. Supported types: shield, shield_2017, xbox_wireless, xbox_wired.",
              joy_type.c_str());
    return false;
  }
  
  // These are the same on all supported controllers
  joystick_dnn_on_button_ = 0;
  joystick_dnn_off_button_ = 1;
  joystick_dnn_left_button_ = 4;
  joystick_dnn_right_button_ = 5;
  return true;
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
      delta_z += 0.1;
      //printCurrentMotion();
      break;
    }
    case 66:
    {
      ROS_WARN_STREAM("\ndown");
      delta_z -= 0.1;
      //printCurrentMotion();
      break;
    }
    case 'a':
    {
      ROS_WARN_STREAM("\nleft");
      delta_x -= 0.1;
      //printCurrentMotion();
      break;
    }
    case 'd':
    {
      ROS_WARN_STREAM("\nright");
      delta_x += 0.1;
      //printCurrentMotion();
      break;
    }
    case 'w':
    {
      ROS_WARN_STREAM("\nforward");
      delta_y += 0.1;
      //printCurrentMotion();
      break;
    }
    case 's':
    {
      ROS_WARN_STREAM("\nbackward");
      delta_y -= 0.1;
      //printCurrentMotion();
      break;
    }
    case 'z':
    {
      ROS_WARN_STREAM("\n z = 0");
      delta_z = 0;
      //printCurrentMotion();
      break;
    }    
    case 'x':
    {
      ROS_WARN_STREAM("\n");
      delta_x = 0;
      //printCurrentMotion();
      break;
    }
    case 'c':
    {
      ROS_WARN_STREAM("\nbackward");
      delta_y = 0;
      //printCurrentMotion();
      break;
    }
  /*
    case 'l':
    {
      ROS_WARN_STREAM("\nrightforward");
      delta_x = 1;
      delta_y = 1;
      //printCurrentMotion();
      break;
    }
    case 'r':
    {
      ROS_WARN_STREAM("\nleftforward");
      delta_x = -1;
      delta_y = 1;
      //printCurrentMotion();
      break;
    }
*/
    case 'r':
    {
      ROS_WARN_STREAM("\nrightround");
      round_flag_r = 1;
      //printCurrentMotion();
      break;
    }
    case 'l':
    {
      ROS_WARN_STREAM("\nleftround");
      round_flag_l = 1;
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
  int toStart = 0;
  ros::init(argc, argv, "offb_main");
  ros::NodeHandle nh;
/************************************************
 Some parameters passed by launch file.
 ***********************************************/
  float spin_rate_ = 20.0f;
  nh.param("spin_rate", spin_rate_, 20.0f);
  ROS_ASSERT(spin_rate_ > 0);
  
  float takeoffVel = 1.0f;
  nh.param("takeoff_vel", takeoffVel, 1.0f);
  
  int timeSpentForTakeoff = 20;
  nh.param("takeoff_time", timeSpentForTakeoff, 20);
  
  std::string joy_type;
  nh.param<std::string>("joy_type", joy_type, "shield_2017");
  if (!setJoystickParams(joy_type))
    return false;
/************************************************
 Define some Subscribers and Publishers.
 ***********************************************/
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                              ("/mavros/state", 100, state_cb);
  if(!state_sub)
    {
        ROS_INFO("Not subscribed to state!");
    }
  else
	ROS_INFO("Subscirbed to state.");
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                 ("/mavros/setpoint_position/local", 100);
      // ENU x-East y-North z-Up
  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
                                 ("/mavros/setpoint_velocity/cmd_vel_unstamped", 100);

  ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                         ("/mavros/local_position/pose", 100, local_pose_cb);
  ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
                                         //("/mavros/local_position/velocity", 100, local_vel_cb);
                                           ("/mavros/global_position/gp_vel", 100, local_vel_cb);
  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 100, joystick_cb); // Subscribe to a JOY (joystick) node if available

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                     ("/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                       ("/mavros/set_mode");


  // The setpoint publishing rate MUST be faster than 2Hz.
  ros::Rate rate(20.0);
  ROS_INFO_STREAM(current_state);
  // Wait for FCU connection.
  while (ros::ok() && current_state.connected) {
      ros::spinOnce();
      rate.sleep();
  }
  delta_x=0; delta_y=0; delta_z=0;


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
      {//ROS_INFO_STREAM(current_state);
          //if( set_mode_client.call(offb_set_mode) &&
        //          offb_set_mode.response.success) {
              ROS_WARN_STREAM("wait for offboard mode......");
        //  }
              updateControlCommandStats();
              last_request = ros::Time::now();
      } else {

          if (!current_state.armed &&
                  (ros::Time::now() - last_request > ros::Duration(5.0))) {
              if( arming_client.call(arm_cmd) &&
                      arm_cmd.response.success) {
                  ROS_WARN_STREAM("Vehicle armed !!!");
                  toStart = 1;
                  updateControlCommandStats();
              }
              last_request = ros::Time::now();
          }
      }

      sendCommand();
      if (toStart == 0)
      {
        vel.linear.x = 0 + delta_x;
        vel.linear.y = 0 + delta_y;
        vel.linear.z = 0 + delta_z;
        
      }
    //start to take off and action
      else if (toStart == 1)
      {
        if(actionOrder == 0)
        {
          ROS_WARN_STREAM("\nup: z = 1, y = -0.2");
          vel.linear.z = takeoffVel + delta_z;
          vel.linear.x = 0 + delta_x;
          vel.linear.y = 0 + delta_y;
          
          timeSpentForTakeoff --;
          if(timeSpentForTakeoff == 0)
            {
                actionOrder++;
                timeSpentForTakeoff = 30;
            }
          
        }
        else if (round_flag_r==1)
        {
          turn_right_round();
        }
        else if (round_flag_l==1)
        {
          turn_left_round();
        }
        else
        {
          vel.linear.z = delta_z;
          vel.linear.x = delta_x;
          vel.linear.y = delta_y;
          
          
        }
      }
    
    /* IMPORTANT!! call this once every while loop*/


	vel_.linear.z = vel.linear.z+altitude_control_val_;
	vel_.linear.y = vel.linear.y+9*north_control_val_;
	vel_.linear.x = vel.linear.x+9*east_control_val_;
	vel_.angular.z = -angular_control_val_;
printf("%f, %f, %f, %f\n", vel_.linear.x, vel_.linear.y, vel_.linear.z, vel_.angular.z);
    local_vel_pub.publish(vel_);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

