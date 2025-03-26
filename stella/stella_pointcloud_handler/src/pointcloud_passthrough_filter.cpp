#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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


class PointCloudPassThroughFilterNode : public rclcpp::Node {
public:
    PointCloudPassThroughFilterNode() : Node("pointcloud_passthrough_filter") {
        sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/camera/depth/color/points", 10,
        std::bind(&PointCloudPassThroughFilterNode::pointCloudCallback, this, std::placeholders::_1));

        pub_pc_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_filtered", 10);
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
        voxel_filter.setLeafSize(0.1, 0.02, 0.15);
        voxel_filter.filter(*cloud_voxel_filtered);

        // 3. PassThrough Filter on Y-axis
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_through_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_voxel_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-1.0, 0.085); // Set floor height (down is y+)
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

        // 6. Noise Non Filter Area Box
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(cloud_angle_filtered);
        crop_box_filter.setMin(Eigen::Vector4f(-1.0, 0.0, 0.18, 1.0)); // Xmin, Ymin, Zmin, 1
        crop_box_filter.setMax(Eigen::Vector4f(1.0, 0.085, 0.4, 1.0));   // Xmax, Ymax, Zmax, 1
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_excluded_region(new pcl::PointCloud<pcl::PointXYZ>);
        crop_box_filter.filter(*cloud_excluded_region);

        // 7. Noise Filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier_removed_region(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud_angle_filtered);
        outrem.setRadiusSearch(0.4); // Radius for outlier removal
        outrem.setMinNeighborsInRadius(75); // Minimum number of neighboring points required
        outrem.filter(*cloud_outlier_removed_region);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud_final_filtered = *cloud_outlier_removed_region + *cloud_excluded_region;

        // 8. Publish Filtered PointCloud2
        sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*cloud_final_filtered, filtered_cloud_msg);
        filtered_cloud_msg.header = cloud_msg->header;
        pub_pc_filtered_->publish(filtered_cloud_msg);
    }


private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_filtered_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudPassThroughFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}