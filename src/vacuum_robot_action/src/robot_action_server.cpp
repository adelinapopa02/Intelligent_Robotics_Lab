#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "vacuum_robot_action/action/charge_battery.hpp" 

namespace vacuum_robot_action
{

class RobotActionServer : public rclcpp::Node
{
public:
    using ChargeBattery = vacuum_robot_action::action::ChargeBattery; 
    using GoalHandleCharge = rclcpp_action::ServerGoalHandle<ChargeBattery>;

    explicit RobotActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("robot", options), current_battery_(5.0)  // Start with 5% battery
    {
        using namespace std::placeholders;

        // Create action server
        action_server_ = rclcpp_action::create_server<ChargeBattery>(
            this,
            "charge_battery",
            std::bind(&RobotActionServer::handle_goal, this, _1, _2),
            std::bind(&RobotActionServer::handle_cancel, this, _1),
            std::bind(&RobotActionServer::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Robot action server started. Current battery: %.1f%%", current_battery_);
    }

private:
    rclcpp_action::Server<ChargeBattery>::SharedPtr action_server_;
    float current_battery_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ChargeBattery::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request with power: %d%%", goal->power);
        
        // Reject if power is invalid (0 or negative, or over 100%)
        if (goal->power <= 0 || goal->power > 100) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: Invalid power level (%d%%)", goal->power);
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        // Reject if battery is already full
        if (current_battery_ >= 100.0) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal: Battery already full (%.1f%%)", current_battery_);
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        // Accept valid requests
        RCLCPP_INFO(this->get_logger(), "Accepting charging goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleCharge> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleCharge> goal_handle)
    {
        // Execute the goal in a new thread
        std::thread{std::bind(&RobotActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleCharge> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing charging goal...");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ChargeBattery::Feedback>();
        auto result = std::make_shared<ChargeBattery::Result>();
        
        // Calculate target battery level: min(100, current + power)
        float target_battery = std::min(100.0f, current_battery_ + goal->power);
        float battery_to_charge = target_battery - current_battery_;
        
        // Calculate time needed: 1 minute to charge from 0% to 100%
        // So time_needed = (battery_to_charge / 100) * 60 seconds
        float total_time_seconds = (battery_to_charge / 100.0f) * 60.0f;
        int num_feedback_updates = static_cast<int>(total_time_seconds);  // 1 feedback per second
        float battery_increment = battery_to_charge / total_time_seconds;
        
        RCLCPP_INFO(this->get_logger(), 
            "Charging from %.1f%% to %.1f%% (%.1f%% increase) over %.1f seconds", 
            current_battery_, target_battery, battery_to_charge, total_time_seconds);
        
        rclcpp::Rate loop_rate(1);  // 1 Hz for feedback
        
        for (int i = 0; i < num_feedback_updates && rclcpp::ok(); ++i) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->header.stamp = this->now();
                result->final_battery_level = static_cast<int32_t>(current_battery_);
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            
            // Update battery level
            current_battery_ += battery_increment;
            
            // Publish feedback
            feedback->header.stamp = this->now();
            feedback->current_battery_level = current_battery_;
            goal_handle->publish_feedback(feedback);
            
            RCLCPP_INFO(this->get_logger(), "Current battery: %.1f%%", current_battery_);
            
            loop_rate.sleep();
        }
        
        // Check if goal is done
        if (rclcpp::ok()) {
            current_battery_ = target_battery;  // Ensure we hit exactly the target
            result->header.stamp = this->now();
            result->final_battery_level = static_cast<int32_t>(current_battery_);
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded! Final battery: %d%%", result->final_battery_level);
        }
    }
};

}  // namespace vacuum_robot_action

RCLCPP_COMPONENTS_REGISTER_NODE(vacuum_robot_action::RobotActionServer)