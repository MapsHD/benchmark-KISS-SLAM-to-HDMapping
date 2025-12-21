/**
 * Livox CustomMsg to PointCloud2 converter for KISS-SLAM
 *
 * Subscribes to /livox/lidar (livox_ros_driver2/msg/CustomMsg)
 * Publishes to /livox/points (sensor_msgs/msg/PointCloud2)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

class LivoxToPC2Node : public rclcpp::Node
{
public:
    LivoxToPC2Node() : Node("livox_to_pc2_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("input_topic", "/livox/lidar");
        this->declare_parameter<std::string>("output_topic", "/livox/points");
        this->declare_parameter<double>("intensity_scale", 1.0);

        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        intensity_scale_ = this->get_parameter("intensity_scale").as_double();

        // Create subscriber and publisher
        subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            input_topic, 10,
            std::bind(&LivoxToPC2Node::livox_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Livox to PointCloud2 converter started");
        RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic.c_str());
    }

private:
    void livox_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        if (msg->points.empty()) {
            return;
        }

        // Create PointCloud2 message
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        // Copy header
        cloud_msg->header = msg->header;

        // Set up fields: x, y, z, intensity, t (timestamp offset)
        cloud_msg->height = 1;
        cloud_msg->width = msg->points.size();
        cloud_msg->is_dense = true;
        cloud_msg->is_bigendian = false;

        // Define point fields
        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        modifier.setPointCloud2Fields(5,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "t", 1, sensor_msgs::msg::PointField::UINT32);

        // Resize to fit all points
        modifier.resize(msg->points.size());

        // Create iterators
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(*cloud_msg, "intensity");
        sensor_msgs::PointCloud2Iterator<uint32_t> iter_t(*cloud_msg, "t");

        // Fill in the point cloud data
        for (const auto& point : msg->points)
        {
            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = point.z;
            *iter_intensity = static_cast<float>(point.reflectivity) * intensity_scale_;
            *iter_t = point.offset_time;

            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_intensity;
            ++iter_t;
        }

        // Publish the converted point cloud
        publisher_->publish(*cloud_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published PointCloud2 with %zu points", msg->points.size());
    }

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    double intensity_scale_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxToPC2Node>());
    rclcpp::shutdown();
    return 0;
}
