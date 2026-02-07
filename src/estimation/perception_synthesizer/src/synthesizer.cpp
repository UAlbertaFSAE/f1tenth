#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <visualization_msgs/msg/marker_array.hpp>
using std::placeholders::_1;
using namespace std::chrono_literals;

class Synthesizer : public rclcpp::Node {
    // thresholds for matching detections
    const rclcpp::Duration TIMESTAMP_THRESHOLD = 100ms;
    const float DISTANCE_THRESHOLD = 0.5; // meters

    const rclcpp::Duration PRUNE_THRESHOLD = 2s; // time after which detections are forgotten

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cone_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_subscription_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr lidar_subscription_;

    struct ConeDetection {
        float x;
        float y;
        float z;
        std::string color;
        rclcpp::Time timestamp;
    };

    std::vector<ConeDetection> camera_detections;
    std::vector<ConeDetection> lidar_detections;

    public:
    Synthesizer() : Node("synthesizer") {
        cone_publisher_ = this->create_publisher<std_msgs::msg::String>("final_cones", 10);

        camera_subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&Synthesizer::camera_callback, this, _1));
        lidar_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("/lidar/cone_cluster_centroids", 10, std::bind(&Synthesizer::lidar_callback, this, _1));
    }

    private:
    void camera_callback(const std_msgs::msg::String::SharedPtr msg) {
        // add to camera_detections vector

        // run every camera callback to call less frequently
        prune_old_detections();
    }

    private:
    void lidar_callback(const visualization_msgs::msg::MarkerArray::SharedPtr centroids) {
        ConeDetection detection;
        detection.x = centroid.pose.position.x;
        detection.y = centroid.pose.position.y;
        detection.z = centroid.pose.position.z;
        detection.timestamp = this->now(); // use timestamp from the message header if available
        lidar_detections.push_back(detection);

        // run match cones here as it runs more frequently
        match_cones();
    }

    void match_cones() {
        for (const auto& lidar_detection : lidar_detections) {
            for (const auto& camera_detection : camera_detections) {
                rclcpp::Duration time_diff = abs(lidar_detection.timestamp - camera_detection.timestamp);
                if (time_diff < TIMESTAMP_THRESHOLD) {
                    float distance = sqrt(pow(lidar_detection.x - camera_detection.x, 2) +
                                          pow(lidar_detection.y - camera_detection.y, 2) +
                                          pow(lidar_detection.z - camera_detection.z, 2));
                    if (distance < DISTANCE_THRESHOLD) {
                        // publish matched cone
                        std_msgs::msg::String msg;
                        msg.data = "Matched cone at: (" + std::to_string(camera_detection.x) + ", " + std::to_string(camera_detection.y) + ", " + std::to_string(camera_detection.z) + ")";
                        cone_publisher_->publish(msg);
                    }
                }
            }
        }
    }

    void prune_old_detections(const rclcpp::Time& timestamp) {
        // remove detections from camera_detections and lidar_detections that are older than a certain threshold
        camera_detections.erase(std::remove_if(camera_detections.begin(), camera_detections.end(),
            [this, timestamp](const ConeDetection& detection) {
                return (timestamp - detection.timestamp) > PRUNE_THRESHOLD;
            }), camera_detections.end());
        lidar_detections.erase(std::remove_if(lidar_detections.begin(), lidar_detections.end(),
            [this, timestamp](const ConeDetection& detection) {
                return (timestamp - detection.timestamp) > PRUNE_THRESHOLD;
            }), lidar_detections.end());
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Synthesizer>());
    rclcpp::shutdown();
    return 0;
}
