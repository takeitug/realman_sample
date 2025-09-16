// api_move_then_vel_minprint.cpp
#include <ros/ros.h>
#include <rm_msgs/MoveJ.h>
#include <rm_msgs/MoveJ_P.h>
#include <rm_msgs/Plan_State.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <array>
#include <sstream>
#include <iomanip>

#define PI 3.14159265358979323846

sensor_msgs::JointState js;
geometry_msgs::Pose ee_pose;

static bool g_plan_ok = false;
void planStateCb(const rm_msgs::Plan_State::ConstPtr& msg) {
  g_plan_ok = msg->state;
}

//joint_states
void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg) {
  js = *msg;
}

//Pose
void eePoseCb(const geometry_msgs::Pose::ConstPtr& msg) {
  ee_pose = *msg;
}

//Joint Position control
void sendMoveJ(ros::Publisher& pub_moveJ, const std::array<double,6>& q_rad, double speed)
{
  rm_msgs::MoveJ cmd;
  cmd.joint.resize(6);
  for (int i=0;i<6;++i) cmd.joint[i] = q_rad[i];
  cmd.speed = speed;
  pub_moveJ.publish(cmd);
}

//Pose control
void sendMoveJ_P(ros::Publisher& pub_moveJP, const geometry_msgs::Pose& pose, double speed)
{
  rm_msgs::MoveJ_P cmd;
  cmd.Pose  = pose;
  cmd.speed = speed;
  pub_moveJP.publish(cmd);
}

//wait
bool waitPlanDone(double timeout_s)
{
  g_plan_ok = false;
  const ros::Time t0 = ros::Time::now();
  ros::Rate r(100);
  while (ros::ok()) {
    if (g_plan_ok) return true;
    if ((ros::Time::now() - t0).toSec() > timeout_s) return false;
    ros::spinOnce();              // 任意タイミング更新
    r.sleep();
  }
  return false;
}

//velocity control switch
void setVelocityEnable(ros::Publisher& pub_enable, bool on)
{
  std_msgs::Bool b; b.data = on;
  for (int i=0;i<3 && ros::ok(); ++i) { pub_enable.publish(b); ros::Duration(0.01).sleep(); }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "api_move_then_vel_minprint");
  ros::NodeHandle nh;

  ros::Publisher  pub_moveJ  = nh.advertise<rm_msgs::MoveJ>  ("/rm_driver/MoveJ_Cmd",   10);
  ros::Publisher  pub_moveJP = nh.advertise<rm_msgs::MoveJ_P>("/rm_driver/MoveJ_P_Cmd", 10);
  ros::Subscriber sub_state  = nh.subscribe("/rm_driver/Plan_State", 10, planStateCb);

  ros::Subscriber sub_js     = nh.subscribe("/joint_states", 50, jointStateCb);
  ros::Subscriber sub_pose   = nh.subscribe("/rm_driver/Pose_State", 50, eePoseCb);

  ros::Publisher  pub_twist  = nh.advertise<geometry_msgs::Twist>("/twist_cmd",   10);
  ros::Publisher  pub_enable = nh.advertise<std_msgs::Bool>  ("/twist_enable",    10);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Duration(2.0).sleep();

  //Joint Position control
  std::array<double,6> q_target = {
    -90 * PI / 180,
    -20 * PI / 180,
    -100 * PI / 180,
    -40 * PI / 180,
    75 * PI / 180,
    -140 * PI / 180
  };
  sendMoveJ(pub_moveJ, q_target, /*speed=*/0.05);
  (void)waitPlanDone(20.0);

  //Pose control
  // geometry_msgs::Pose ee_target;
  // ee_target.position.x = -0.317239;
  // ee_target.position.y =  0.120903;
  // ee_target.position.z =  0.255765 + 0.04;
  // ee_target.orientation.x = -0.983404;
  // ee_target.orientation.y = -0.178432;
  // ee_target.orientation.z =  0.032271;
  // ee_target.orientation.w =  0.006129;
  // sendMoveJ_P(pub_moveJP, ee_target, /*speed=*/0.05);
  // (void)waitPlanDone(20.0);

  //enable velocity control
  setVelocityEnable(pub_enable, true);

  //velocity control sample
  {
    const double vz = 0.03;            // m/s
    const double dur = 1.0;            // s
    geometry_msgs::Twist tw; tw.linear.z = vz;
    ros::Rate rate(100.0);
    const ros::Time t0 = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - t0).toSec() < dur) {
      pub_twist.publish(tw);

      ros::spinOnce();
      ROS_INFO("joint= [%.4f %.4f %.4f %.4f %.4f %.4f]",js.position[0],js.position[1],js.position[2],js.position[3],js.position[4],js.position[5]);
      ROS_INFO("pose = [%.4f %.4f %.4f %.4f %.4f %.4f %.4f]",ee_pose.position.x,ee_pose.position.y,ee_pose.position.z,ee_pose.orientation.x,ee_pose.orientation.y,ee_pose.orientation.z,ee_pose.orientation.w);

      rate.sleep();
    }

    //stop
    geometry_msgs::Twist zero; pub_twist.publish(zero);
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }

  //disable velocity control
  setVelocityEnable(pub_enable, false);

  ros::shutdown();
  return 0;
}
