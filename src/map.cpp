#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include "std_srvs/srv/trigger.hpp"

class IgLioMapNode : public rclcpp::Node {
public:
    IgLioMapNode() : Node("ig_lio_map_node") {
        this->getParams();
        
        this->keyframe_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "keyframe_scan", 10, std::bind(&IgLioMapNode::keyframeCallback, this, std::placeholders::_1));
        
        this->save_map_service_ = this->create_service<std_srvs::srv::Trigger>(
            "save_map", std::bind(&IgLioMapNode::saveMapService, this, std::placeholders::_1, std::placeholders::_2));
        
    this->world_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_map", 100);

        this->ig_lio_map = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }

private:
    void getParams() {
        
        
        this->declare_parameter<std::string>("map/save_map_path", "/");
        this->get_parameter("map/save_map_path", this->save_map_path_);
        this->declare_parameter<std::string>("map/map_name", "ig_lio_map");
        this->get_parameter("map/map_name", this->map_name_);
        this->declare_parameter<std::string>("map/map_frame", "world");
        this->get_parameter("map/map_frame", this->map_frame_);
    }

    void keyframeCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        *this->ig_lio_map += *cloud;

        sensor_msgs::msg::PointCloud2 world_ros;
        pcl::toROSMsg(*this->ig_lio_map, world_ros);
        world_ros.header.stamp = msg->header.stamp;
        world_ros.header.frame_id = this->map_frame_;
        this->world_pub->publish(world_ros);
    }

    void saveMapService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::string file_path = this->save_map_path_ + "/" + this->map_name_+ ".pcd";
        if (pcl::io::savePCDFileBinary(file_path, *this->ig_lio_map) == 0) {
            response->success = true;
            response->message = "Map saved successfully to " + file_path;
        } else {
            response->success = false;
            response->message = "Failed to save the map.";
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_pub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ig_lio_map;
    std::string save_map_path_;
    std::string map_frame_;
    std::string map_name_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IgLioMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
