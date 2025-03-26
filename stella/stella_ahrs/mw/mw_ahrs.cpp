#include "mw_ahrs.hpp"
#include "mw_ahrsX1_def.hpp"

static bool AHRS = false;

namespace ntrex
{
  void MwAhrsRosDriver::StartReading()
  {
    AHRS = true;
    sleep(1);
    reading_thread_ = std::thread(&MwAhrsRosDriver::MwAhrsRead, this);
  }

  void MwAhrsRosDriver::StopReading()
  {
    if (AHRS)
    {
      AHRS = false;
      sleep(1);
      if (reading_thread_.joinable())
      {
        reading_thread_.join();
      }
    }
  }

  void MwAhrsRosDriver::StartPubing()
  {
    if (AHRS)
      publisher_thread_ = std::thread(&MwAhrsRosDriver::publish_topic, this);
  }

  void MwAhrsRosDriver::StopPubing()
  {
    if (publisher_thread_.joinable())
      publisher_thread_.join();
  }

  void MwAhrsRosDriver::MW_AHRS_Covariance(void)
  {
    imu_data_raw_msg = sensor_msgs::msg::Imu();
    imu_data_msg = sensor_msgs::msg::Imu();
    imu_magnetic_msg = sensor_msgs::msg::MagneticField();
    imu_yaw_msg = std_msgs::msg::Float64();

    linear_acceleration_cov = linear_acceleration_stddev_ * linear_acceleration_stddev_;
    angular_velocity_cov = angular_velocity_stddev_ * angular_velocity_stddev_;
    magnetic_field_cov = magnetic_field_stddev_ * magnetic_field_stddev_;
    orientation_cov = orientation_stddev_ * orientation_stddev_;

    imu_data_raw_msg.linear_acceleration_covariance[0] =
        imu_data_raw_msg.linear_acceleration_covariance[4] =
            imu_data_raw_msg.linear_acceleration_covariance[8] =
                imu_data_msg.linear_acceleration_covariance[0] =
                    imu_data_msg.linear_acceleration_covariance[4] =
                        imu_data_msg.linear_acceleration_covariance[8] =
                            linear_acceleration_cov;

    imu_data_raw_msg.angular_velocity_covariance[0] =
        imu_data_raw_msg.angular_velocity_covariance[4] =
            imu_data_raw_msg.angular_velocity_covariance[8] =
                imu_data_msg.angular_velocity_covariance[0] =
                    imu_data_msg.angular_velocity_covariance[4] =
                        imu_data_msg.angular_velocity_covariance[8] =
                            angular_velocity_cov;

    imu_data_msg.orientation_covariance[0] =
        imu_data_msg.orientation_covariance[4] =
            imu_data_msg.orientation_covariance[8] =
                orientation_cov;

    imu_magnetic_msg.magnetic_field_covariance[0] =
        imu_magnetic_msg.magnetic_field_covariance[4] =
            imu_magnetic_msg.magnetic_field_covariance[8] =
                magnetic_field_cov;
  }

  void MwAhrsRosDriver::MwAhrsRead()
  {
    while (AHRS)
    {
      unsigned char data[8];

      if (MW_AHRS_Read(data))
      {
        switch ((int)(unsigned char)data[1])
        {
        case ACC:
          acc_value[0] = (int16_t)(((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8)) / 1000.0;
          acc_value[1] = (int16_t)(((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8)) / 1000.0;
          acc_value[2] = (int16_t)(((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8)) / 1000.0;

          imu_data_raw_msg.linear_acceleration.x = imu_data_msg.linear_acceleration.x =
              acc_value[0] * convertor_g2a;
          imu_data_raw_msg.linear_acceleration.y = imu_data_msg.linear_acceleration.y =
              acc_value[1] * convertor_g2a;
          imu_data_raw_msg.linear_acceleration.z = imu_data_msg.linear_acceleration.z =
              acc_value[2] * convertor_g2a;

          break;

        case GYO:
          gyr_value[0] = (int16_t)(((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8)) / 10.0;
          gyr_value[1] = (int16_t)(((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8)) / 10.0;
          gyr_value[2] = (int16_t)(((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8)) / 10.0;

          imu_data_raw_msg.angular_velocity.x = imu_data_msg.angular_velocity.x =
              gyr_value[0] * convertor_d2r;
          imu_data_raw_msg.angular_velocity.y = imu_data_msg.angular_velocity.y =
              gyr_value[1] * convertor_d2r;
          imu_data_raw_msg.angular_velocity.z = imu_data_msg.angular_velocity.z =
              gyr_value[2] * convertor_d2r;

          break;

        case DEG:
          deg_value[0] = (int16_t)(((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8)) / 100.0;
          deg_value[1] = (int16_t)(((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8)) / 100.0;
          deg_value[2] = (int16_t)(((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8)) / 100.0;

          roll = deg_value[0] * convertor_d2r;
          pitch = mag_value[1] * convertor_d2r;
          yaw = deg_value[2] * convertor_d2r;

          tf_orientation = Euler2Quaternion(roll, pitch, yaw);

          imu_yaw_msg.data = deg_value[2];

          imu_data_msg.orientation.x = tf_orientation.x();
          imu_data_msg.orientation.y = tf_orientation.y();
          imu_data_msg.orientation.z = tf_orientation.z();
          imu_data_msg.orientation.w = tf_orientation.w();

          break;

        case MAG:
          mag_value[0] = (int16_t)(((int)(unsigned char)data[2] | (int)(unsigned char)data[3] << 8)) / 10.0;
          mag_value[1] = (int16_t)(((int)(unsigned char)data[4] | (int)(unsigned char)data[5] << 8)) / 10.0;
          mag_value[2] = (int16_t)(((int)(unsigned char)data[6] | (int)(unsigned char)data[7] << 8)) / 10.0;

          imu_magnetic_msg.magnetic_field.x = mag_value[0] / convertor_ut2t;
          imu_magnetic_msg.magnetic_field.y = mag_value[1] / convertor_ut2t;
          imu_magnetic_msg.magnetic_field.z = mag_value[2] / convertor_ut2t;

          break;
        }
      }
    }
  }

  void MwAhrsRosDriver::publish_topic()
  {
    rclcpp::Rate rate(1000);

    while (rclcpp::ok() && AHRS)
    {
      rclcpp::Time now = this->get_clock()->now();

      imu_data_raw_msg.header.stamp = imu_data_msg.header.stamp = imu_magnetic_msg.header.stamp = now;
      imu_data_raw_msg.header.frame_id = imu_data_msg.header.frame_id = imu_magnetic_msg.header.frame_id = frame_id_;

      imu_data_raw_pub_->publish(std::move(imu_data_raw_msg));
      imu_data_pub_->publish(std::move(imu_data_msg));
      imu_mag_pub_->publish(std::move(imu_magnetic_msg));
      imu_yaw_pub_->publish(std::move(imu_yaw_msg));

      if (publish_tf_)
      {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = parent_frame_id_;
        tf.child_frame_id = frame_id_;
        tf.transform.translation.x = 0.0;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = imu_data_msg.orientation;

        broadcaster_->sendTransform(tf);
      }
      rate.sleep();
    }
  }

  tf2::Quaternion MwAhrsRosDriver::Euler2Quaternion(float roll, float pitch, float yaw)
  {
    float qx = (sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) -
               (cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2));
    float qy = (cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2)) +
               (sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2));
    float qz = (cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2)) -
               (sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2));
    float qw = (cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2)) +
               (sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2));

    tf2::Quaternion q(qx, qy, qz, qw);
    return q;
  }

  bool MwAhrsRosDriver::MW_AHRS_Setting()
  {
    bool res = true;

    long product_id = 0, software_ver = 0, hardware_ver = 0, function_ver = 0;

    long sync_port = CI_USB, sync_period = 10, sync_trmode = CI_Binary, sync_data = 15, FlashWrite = 1;

    res &= MW_AHRS_GetValI(product_id,   CI_PRODUCT_ID);
    res &= MW_AHRS_GetValI(software_ver, CI_SW_VERSION);
    res &= MW_AHRS_GetValI(hardware_ver, CI_HW_VERSION);
    res &= MW_AHRS_GetValI(function_ver, CI_FN_VERSION);

    RCLCPP_INFO(this->get_logger(), "product_id   : %ld \n", product_id);
    RCLCPP_INFO(this->get_logger(), "software_ver : %ld \n", software_ver);
    RCLCPP_INFO(this->get_logger(), "hardware_ver : %ld \n", hardware_ver);
    RCLCPP_INFO(this->get_logger(), "function_ver : %ld \n", function_ver);

    res &= MW_AHRS_SetValI(sync_port,   CI_SYNC_PORT);
    res &= MW_AHRS_SetValI(sync_period, CI_SYNC_PERIOD);
    res &= MW_AHRS_SetValI(sync_trmode, CI_SYNC_TRMODE);
    res &= MW_AHRS_SetValI(sync_data,   CI_SYNC_DATA);
    // res &= MW_AHRS_SetValI(FlashWrite,  CI_SYS_COMMAND);

    // res &= MW_AHRS_NvicReset ();

    return res;
  }

  MwAhrsRosDriver::MwAhrsRosDriver(char *port, int baud_rate) : Node("MW_AHRS_ROS2")
  {
    bool res = false;

    res = MW_AHRS_Connect(port, baud_rate);

    if(res) res = MW_AHRS_Setting();

    if (res)
    {
      this->declare_parameter("linear_acceleration_stddev", 0.0);
      this->declare_parameter("angular_velocity_stddev", 0.0);
      this->declare_parameter("magnetic_field_stddev", 0.0);
      this->declare_parameter("orientation_stddev", 0.0);

      this->get_parameter("linear_acceleration_stddev", linear_acceleration_stddev_);
      this->get_parameter("angular_velocity_stddev", angular_velocity_stddev_);
      this->get_parameter("magnetic_field_stddev", magnetic_field_stddev_);
      this->get_parameter("orientation_stddev", orientation_stddev_);

      MW_AHRS_Covariance();

      StartReading();

      auto qos = rclcpp::QoS(rclcpp::KeepLast(10)) .reliable() .durability_volatile();

      imu_data_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", qos);
      imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
      imu_mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", qos);
      imu_yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("imu/yaw", qos);

      StartPubing();
      RCLCPP_INFO(this->get_logger(), "MW-AHRS ROS Init Success");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "MW-AHRS ROS Init Fail");
    }
  }

  MwAhrsRosDriver::~MwAhrsRosDriver()
  {
    long stop_data = 0;

    MW_AHRS_SetValI(stop_data, CI_SYNC_DATA);
    StopReading();
    StopPubing();
    MW_AHRS_DisConnect();
  }
}
