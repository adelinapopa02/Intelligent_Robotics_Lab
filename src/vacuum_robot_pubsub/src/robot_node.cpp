#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "vacuum_robot_pubsub/msg/charge_status.hpp"

using namespace std::chrono_literals;

class RobotNode : public rclcpp::Node
{
public:
    RobotNode() : Node("robot"), current_room_index_(0), battery_level_(100.0)
    {
        // Create publisher on /charge_status topic
        publisher_ = this->create_publisher<vacuum_robot_pubsub::msg::ChargeStatus>(
            "/charge_status", 10);
        
        // Create timer to publish at 5Hz (200ms period)
        timer_ = this->create_wall_timer(
            200ms, std::bind(&RobotNode::timer_callback, this));
        
        // Initialize the 5 rooms
        rooms_ = {
            {1, "Robot Vision Lab"},
            {2, "SSL Lab"},
            {3, "Neurorobotics Lab"},
            {4, "IAS-Lab"},
            {5, "Autonomous Robotics Lab"}
        };
        
        RCLCPP_INFO(this->get_logger(), "Robot node started");
    }

private:
    void timer_callback()
    {
        // Create message
        auto message = vacuum_robot_pubsub::msg::ChargeStatus();
        
        // Get current room
        auto current_room = rooms_[current_room_index_];
        message.room_id = current_room.id;
        message.room_name = current_room.name;
        message.battery_level = battery_level_;
        
        // Publish message
        publisher_->publish(message);
        
        RCLCPP_INFO(this->get_logger(), 
            "Publishing: Room ID=%d, Room='%s', Battery=%.1f%%",
            message.room_id, message.room_name.c_str(), message.battery_level);
        
        // Simulate battery drain
        battery_level_ -= 0.5;
        if (battery_level_ < 0.0) {
            battery_level_ = 0.0;
        }
        
        // Simulate room change every 10 seconds (50 messages at 5Hz)
        static int message_count = 0;
        message_count++;
        if (message_count >= 50) {
            if (battery_level_ > 0.0) {
                current_room_index_ = (current_room_index_ + 1) % rooms_.size();
                RCLCPP_INFO(this->get_logger(), "Moving to next room...");
            }
            message_count = 0;
        }
    }
    
    struct Room {
        int id;
        std::string name;
    };
    
    rclcpp::Publisher<vacuum_robot_pubsub::msg::ChargeStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Room> rooms_;
    size_t current_room_index_;
    float battery_level_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNode>());
    rclcpp::shutdown();
    return 0;
}