#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "vacuum_robot_pubsub/msg/charge_status.hpp"

class ChargingStationNode : public rclcpp::Node
{
public:
    ChargingStationNode() : Node("charging_station")
    {
        // Create subscriber to /charge_status topic
        subscription_ = this->create_subscription<vacuum_robot_pubsub::msg::ChargeStatus>(
            "/charge_status", 10,
            std::bind(&ChargingStationNode::topic_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Charging station node started");
        RCLCPP_INFO(this->get_logger(), "Waiting for robot status messages...");
    }

private:
    void topic_callback(const vacuum_robot_pubsub::msg::ChargeStatus::SharedPtr msg)
    {
        // Print received information
        RCLCPP_INFO(this->get_logger(), 
            "Received: Room ID=%d, Room='%s', Battery=%.1f%%",
            msg->room_id, msg->room_name.c_str(), msg->battery_level);
        
        // Warning if battery is low
        if ((msg->battery_level < 20.0) && (msg->battery_level > 0.0)) {
            RCLCPP_WARN(this->get_logger(), "WARNING: Battery level is LOW!");
        }
        // Critical warning if battery is depleted
        if (msg->battery_level == 0.0) {
            RCLCPP_ERROR(this->get_logger(), "CRITICAL: Battery DEPLETED! Robot needs charging!");
        }
    }
    
    rclcpp::Subscription<vacuum_robot_pubsub::msg::ChargeStatus>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChargingStationNode>());
    rclcpp::shutdown();
    return 0;
}