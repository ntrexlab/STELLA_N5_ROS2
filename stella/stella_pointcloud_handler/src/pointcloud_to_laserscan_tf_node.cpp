#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <limits>


class PointCloudToLaserScanNodeTF : public rclcpp::Node {
public:
    PointCloudToLaserScanNodeTF() : Node("pointcloud_to_laserscan_tf"), tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)) {
        sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/camera/depth/color/points", 10,
        std::bind(&PointCloudToLaserScanNodeTF::pointCloudCallback, this, std::placeholders::_1));

        pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_pointcloud", rclcpp::QoS(rclcpp::KeepLast(10)));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        // 1. Convert ROS message to PCL cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // 2. VoxelGrid Filter (DownSampling)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.03, 0.03, 0.1);
        voxel_filter.filter(*cloud_voxel_filtered);

        // 3. PassThrough Filter on Y-axis
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_through_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_voxel_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-0.8, 0.11);
        pass.filter(*cloud_pass_through_filtered);

        // 4. PassThrough Filter on Z-axis (distance)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_distance_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_distance;
        pass_distance.setInputCloud(cloud_pass_through_filtered);
        pass_distance.setFilterFieldName("z");
        pass_distance.setFilterLimits(0.18, 2.0); // Intel RealSense D435i distance
        pass_distance.filter(*cloud_distance_filtered);

        // 5. Angle Filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_angle_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_angle_filtered->points.reserve(cloud_distance_filtered->points.size());
        for (const auto& point : cloud_distance_filtered->points) {
            float angle = std::atan2(point.x, point.z);
            if (angle >= -43.0 * M_PI / 180.0 && angle <= 43.0 * M_PI / 180.0)
            {
                cloud_angle_filtered->points.push_back(point);
            }
        }
        cloud_angle_filtered->width = cloud_angle_filtered->points.size();
        cloud_angle_filtered->height = 1;
        cloud_angle_filtered->is_dense = true;

        // 6. Noise Filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud_angle_filtered);
        outrem.setRadiusSearch(0.1); // Radius for outlier removal
        outrem.setMinNeighborsInRadius(5); // Minimum number of neighboring points required
        outrem.filter(*cloud_final_filtered);

        // 7. Generation Laserscan by Filtered PointCloud
        sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*cloud_final_filtered, filtered_cloud_msg);

        auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        scan_msg->header = cloud_msg->header;
        scan_msg->header.frame_id = "depth_scan_frame";

        double angle_min = -0.76;
        double angle_max = 0.76;
        double angle_increment = 0.01745;
        double scan_time = 1.0 / 30.0;
        double range_min = std::numeric_limits<double>::min();
        double range_max = std::numeric_limits<double>::max();
        double inf_epsilon = 1.0;
        bool use_inf = true;

        scan_msg->angle_min = angle_min;
        scan_msg->angle_max = angle_max;
        scan_msg->angle_increment = angle_increment;
        scan_msg->time_increment = 0.0;
        scan_msg->scan_time = scan_time;
        scan_msg->range_min = range_min;
        scan_msg->range_max = range_max;

        uint32_t ranges_size = std::ceil(
            (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

        if (use_inf) {
            scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
        } else {
            scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon);
        }

        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(filtered_cloud_msg, "x"),
             iter_y(filtered_cloud_msg, "y"), iter_z(filtered_cloud_msg, "z");
             iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
                RCLCPP_DEBUG(this->get_logger(), "rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
                continue;
            }

            double range = std::hypot(*iter_x, *iter_z);
            if (range < range_min) {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
                    range, range_min, *iter_x, *iter_y, *iter_z);
                continue;
            }
            if (range > range_max) {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
                    range, range_max, *iter_x, *iter_y, *iter_z);
                continue;
            }

            double angle = std::atan2(*iter_x, *iter_z);
            if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "rejected for angle %f not in range (%f, %f)\n",
                    angle, scan_msg->angle_min, scan_msg->angle_max);
                continue;
            }

            int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
            if (range < scan_msg->ranges[index]) {
                scan_msg->ranges[index] = range;
            }
        }

        broadcastLaserScanTransform(cloud_msg->header.stamp);
        pub_scan_->publish(*scan_msg);
    }

    void broadcastLaserScanTransform(rclcpp::Time stamp)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = "camera_depth_optical_frame";
        transform_stamped.child_frame_id = "depth_scan_frame";

        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;

        tf2::Quaternion rotation_quat;
        tf2::Quaternion q_orig, q_rot_y, q_rot_z, q_combined;

        q_orig.setRPY(0.0, 0.0, 0.0);
        q_rot_y.setRPY(0.0, -M_PI/2.0, 0.0);
        q_rot_z.setRPY(0.0, 0.0, -M_PI/2.0);

        q_combined = q_rot_y * q_orig;
        q_combined = q_rot_z * q_combined;
        transform_stamped.transform.rotation.x = q_combined.x();
        transform_stamped.transform.rotation.y = q_combined.y();
        transform_stamped.transform.rotation.z = q_combined.z();
        transform_stamped.transform.rotation.w = q_combined.w();


        tf_broadcaster_->sendTransform(transform_stamped);
    }


private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToLaserScanNodeTF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}