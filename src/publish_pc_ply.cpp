//
// Created by olagh48652 on 9/7/24.
//
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCloudPublisher : public rclcpp::Node
{
public:
    PointCloudPublisher() : Node("point_cloud_publisher")
    {
        // Create a publisher for the point cloud
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);

        // Load the PLY file and publish the point cloud
        publishPointCloud();
    }

private:
    void publishPointCloud()
    {
        // Create a PCL PointCloud object for the input with float fields
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        // Create a PCL PointCloud object for the output with RGB
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Load the PLY file
        if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/olagh48652/rss_paper/data/train/point_cloud/iteration_7000/point_cloud.ply", *cloud_in) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read the PLY file.");
            return;
        }

//        // Resize the output cloud to match the input cloud size
//        cloud_out->points.resize(cloud_in->points.size());

//        // Convert from pcl::PointXYZ to pcl::PointXYZRGB
//        for (size_t i = 0; i < cloud_in->points.size(); ++i)
//        {
//            // Copy position data
//            cloud_out->points[i].x = cloud_in->points[i].x;
//            cloud_out->points[i].y = cloud_in->points[i].y;
//            cloud_out->points[i].z = cloud_in->points[i].z;
//
//            // Convert float RGB values to uchar
////            cloud_out->points[i].r = static_cast<uint8_t>(std::min(std::max(cloud_in->points[i].f_dc_0 * 255.0f, 0.0f), 255.0f));
////            cloud_out->points[i].g = static_cast<uint8_t>(std::min(std::max(cloud_in->points[i].f_dc_1 * 255.0f, 0.0f), 255.0f));
////            cloud_out->points[i].b = static_cast<uint8_t>(std::min(std::max(cloud_in->points[i].f_dc_2 * 255.0f, 0.0f), 255.0f));
//        }

        // Convert the PCL point cloud to a ROS2 PointCloud2 message
        sensor_msgs::msg::PointCloud2 output;
//        pcl::toROSMsg(*cloud_out, output);
        pcl::toROSMsg(*cloud_in, output);
        output.header.frame_id = "map";  // Set the frame ID to match RViz2

        // Publish the point cloud
        publisher_->publish(output);

//        RCLCPP_INFO(this->get_logger(), "Published point cloud with %zu points.", cloud_out->points.size());
        RCLCPP_INFO(this->get_logger(), "Published point cloud with %zu points.", cloud_in->points.size());
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
