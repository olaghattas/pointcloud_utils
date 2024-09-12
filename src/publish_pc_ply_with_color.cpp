//
// Created by olagh48652 on 9/7/24.
//
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "happly.h"
#include <filesystem>
#include <iostream>
#include <vector>
#include <string>



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
    // formula from Zero order Spherical Harmonics (f_dc_0, f_dc_1, f_dc_2), which dictate what color the single splat should have by using a specific mathematical formula to extract the output RGB value for the rendering;
    std::tuple<double, double, double> calculate_rgb(double f_dc_0, double f_dc_1, double f_dc_2) {
        const double SH_C0 = 0.28209479177387814;
        double r = 0.5 + SH_C0 * f_dc_0;
        double g = 0.5 + SH_C0 * f_dc_1;
        double b = 0.5 + SH_C0 * f_dc_2;

        // Clamp values to ensure they are not less than 0
        r = std::max(r, 0.0);
        g = std::max(g, 0.0);
        b = std::max(b, 0.0);

        // Rescale to ensure the largest value is 1.0
        double max_channel = std::max({r, g, b});
        if (max_channel > 1.0) {
            r /= max_channel;
            g /= max_channel;
            b /= max_channel;
        }

        return std::make_tuple(r, g, b);
    }

    void publishPointCloud()
    {
//        std::filesystem::path filePath("/home/olagh48652/rss_paper/data/train/point_cloud/iteration_7000/point_cloud.ply");
        std::filesystem::path filePath("/home/olagh48652/rss_paper/output_run2/b41e5e6f-8/point_cloud/iteration_30000/point_cloud.ply");

        if (std::filesystem::exists(filePath)) {
            std::cout << "File exists!" << std::endl;
        } else {
            std::cout << "File does not exist." << std::endl;
        }
        // Load the PLY file
        happly::PLYData plyData(filePath);

        // Extract vertex positions
        std::vector<std::array<double, 3>> vertices = plyData.getVertexPositions();

        // Extract custom fields (e.g., color components f_dc_0, f_dc_1, f_dc_2)
        std::vector<std::array<double, 3>> color = plyData.getVertexColorsfdc();

        // Create a PointCloud2 message
        sensor_msgs::msg::PointCloud2 pointcloud_msg;
        pointcloud_msg.header.frame_id = "map";
        pointcloud_msg.header.stamp = this->now();

        // Define the number of points in the point cloud
        int num_points = vertices.size();

        // Set the fields of the PointCloud2 message
        pointcloud_msg.height = 1;  // Unorganized point cloud
        pointcloud_msg.width = num_points;
        pointcloud_msg.is_dense = false;  // Indicates if there are invalid points

        // Define the fields of the PointCloud2 message
        sensor_msgs::PointCloud2Modifier pcd_modifier(pointcloud_msg);
        pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

        // Allocate memory for the PointCloud2 message
        pcd_modifier.resize(num_points);

        // Create an iterator for the PointCloud2 message
        sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pointcloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pointcloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pointcloud_msg, "b");

        double R, G, B;


        for (int i = 0; i < num_points; ++i)
        {
            *iter_x = static_cast<float>(vertices[i][0]);
            *iter_y = static_cast<float>(vertices[i][1]);
            *iter_z = static_cast<float>(vertices[i][2]);

            // Set the color for the point cloud (Example RGB values)
            std::tie(R, G, B) = calculate_rgb(color[i][0], color[i][1], color[i][2]);

            *iter_r = static_cast<uint8_t>(R*255);
            *iter_g = static_cast<uint8_t>(G*255);
            *iter_b = static_cast<uint8_t>(B*255);


            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_r;
            ++iter_g;
            ++iter_b;
        }


        // Publish the point cloud
        publisher_->publish(pointcloud_msg);

        RCLCPP_INFO(this->get_logger(), "Published point cloud with %d points.", num_points);
//        RCLCPP_INFO(this->get_logger(), "Published point cloud with %zu points.", cloud_out->points.size());
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
