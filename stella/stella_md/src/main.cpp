#include "main.hpp"
#include "MW_value.hpp"
#include "MW_serial.hpp"
#include "stella.hpp"
#include <math.h>

#define convertor_d2r (M_PI / 180.0)

static bool RUN = false;

inline int Limit_i (int v, int lo, int hi)
{
	if(abs(v) > lo && abs(v) < hi) return v;
	
  else return 0;
}

inline double pulse2meter()
{
  double meter= ((2 * M_PI * Differential_MobileRobot.wheel_radius) / Differential_MobileRobot.gear_ratio / MyMotorConfiguration.encoder_ppr[0]);

  return meter; //엔코더 역상
}

stellaN1_node::stellaN1_node() : Node("stella_md_node")
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  ahrs_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>("imu/yaw", 1, std::bind(&stellaN1_node::ahrs_yaw_data_callback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&stellaN1_node::command_velocity_callback, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  Serial_timer = this->create_wall_timer(1ms, std::bind(&stellaN1_node::serial_callback, this));
}

stellaN1_node::~stellaN1_node()
{
  MW_Serial_DisConnect();
}

void stellaN1_node::ahrs_yaw_data_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  ahrs_yaw = msg->data;
}

void stellaN1_node::command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
{
  if(RUN)
  {
    goal_linear_velocity_ = cmd_vel_msg->linear.x;
    goal_angular_velocity_ = cmd_vel_msg->angular.z ;

    dual_m_command(dual_m_command_select::m_lav, goal_linear_velocity_, goal_angular_velocity_);
  }
}

void stellaN1_node::serial_callback()
{
  if(RUN)
  {
    Motor_MonitoringCommand(channel_1, _position);
    Motor_MonitoringCommand(channel_2, _position);

    update_odometry();
  }
}

bool stellaN1_node::update_odometry()
{

  delta_left = Limit_i((MyMotorCommandReadValue.position[channel_1] - left_encoder_prev), 0, 15000) * pulse2meter();
  delta_right = Limit_i((MyMotorCommandReadValue.position[channel_2] - right_encoder_prev), 0, 15000) * pulse2meter();
  
  //로봇 기구학 적용및 센서퓨전
  delta_s  = (delta_right + delta_left) / 2.0 ;
  delta_th = (ahrs_yaw * convertor_d2r); 

/*
  th 값은 AHRS YAW 값을 참조하여 아래의 식은 주석으로 처리한다.
  delta_th = (delta_right - delta_left) / Differential_MobileRobot.axle_length;
*/

  delta_x  = delta_s * cos(delta_th);
  delta_y  = delta_s * sin(delta_th);
  
  x += delta_x;
  y += delta_y;

  nav_msgs::msg::Odometry odom;

  tf2::Quaternion Quaternion;
  Quaternion.setRPY(0, 0, delta_th);

  odom.pose.pose.orientation.x = Quaternion.x();
  odom.pose.pose.orientation.y = Quaternion.y();
  odom.pose.pose.orientation.z = Quaternion.z();
  odom.pose.pose.orientation.w = Quaternion.w();

  geometry_msgs::msg::TransformStamped t;
  
  rclcpp::Time time_now = this->now();
  
  t.header.stamp = time_now;
  t.header.frame_id = "odom";
  t.child_frame_id = "base_footprint";

  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = 0.0;

  t.transform.rotation.x = Quaternion.x();
  t.transform.rotation.y = Quaternion.y();
  t.transform.rotation.z = Quaternion.z();
  t.transform.rotation.w = Quaternion.w();

  odom_broadcaster->sendTransform(t);

  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;

  odom.child_frame_id = "base_footprint";
  odom.twist.twist.linear.x = goal_linear_velocity_;
  odom.twist.twist.angular.z = goal_angular_velocity_;

  odom.header.stamp = time_now;
  odom_pub_->publish(odom);

  left_encoder_prev = MyMotorCommandReadValue.position[channel_1];
  right_encoder_prev = MyMotorCommandReadValue.position[channel_2];
  return true;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  MW_Serial_Connect("/dev/MW", 115200);

  if(Robot_Setting(::N1)) RUN = true;
  Robot_Fault_Checking_RESET();
  
  rclcpp::spin(std::make_shared<stellaN1_node>());

  rclcpp::shutdown();
  return 0;
}
