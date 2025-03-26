#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <iostream>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "mw_serial.hpp"

#define ACC 0x33
#define GYO 0x34
#define DEG 0x35
#define MAG 0x36

#define convertor_g2a 9.80665        // linear_acceleration (g to m/s^2)
#define convertor_d2r (M_PI / 180.0) // angular_velocity (degree to radian)
#define convertor_ut2t 1000000       // magnetic_field (uT to Tesla)
#define convertor_c 1.0              // temperature (celsius)

using namespace std::chrono_literals;

static float acc_value[3] = {
    0,
},
             gyr_value[3] = {
                 0,
},
             deg_value[3] = {
                 0,
},
             mag_value[3] = {
                 0,
};


namespace ntrex
{
    class MwAhrsRosDriver : public rclcpp::Node
    {
    public:
        sensor_msgs::msg::Imu imu_data_raw_msg;
        sensor_msgs::msg::Imu imu_data_msg;
        sensor_msgs::msg::MagneticField imu_magnetic_msg;
        std_msgs::msg::Float64 imu_yaw_msg;

        tf2::Quaternion tf_orientation;

    public:
        double linear_acceleration_stddev_, angular_velocity_stddev_, magnetic_field_stddev_, orientation_stddev_;
        double linear_acceleration_cov, angular_velocity_cov, magnetic_field_cov, orientation_cov;
        double roll, pitch, yaw;

    private:
        bool publish_tf_;
        std::string parent_frame_id_;
        std::string frame_id_;
        std::mutex _lockAHRS;

    public:
        MwAhrsRosDriver(char *port, int baud_rate);
        ~MwAhrsRosDriver();

        void StartReading();
        void StopReading();
        void StartPubing();
        void StopPubing();

        void MW_AHRS_Covariance();

        std::thread reading_thread_, publisher_thread_;

        void MwAhrsRead();
        tf2::Quaternion Euler2Quaternion(float roll, float pitch, float yaw);
        void publish_topic();
        bool MW_AHRS_Setting ();

    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_raw_pub_, imu_data_pub_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr imu_mag_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr imu_yaw_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    };
}