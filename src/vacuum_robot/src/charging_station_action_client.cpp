#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "vacuum_robot/action/charge_battery.hpp"

namespace vacuum_robot
{

class ChargingStationActionClient : public rclcpp::Node
{
public:
    using ChargeBattery = vacuum_robot::action::ChargeBattery;
    using GoalHandleCharge = rclcpp_action::ClientGoalHandle<ChargeBattery>;

    explicit ChargingStationActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("charging_station", options)
    {
        // Create action client
        action_client_ = rclcpp_action::create_client<ChargeBattery>(this, "charge_battery");
        
        // Create timer to send goal after 1 second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ChargingStationActionClient::send_goal, this));
    }

    void send_goal()
    {
        timer_->cancel();  // Only send goal once
        
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = ChargeBattery::Goal();
        goal_msg.header.stamp = this->now();
        goal_msg.power = 70;  // Provide x% charging power (must be valid to be accepted by server)
        
        RCLCPP_INFO(this->get_logger(), "Sending goal: power = %d%%", goal_msg.power);

        auto send_goal_options = rclcpp_action::Client<ChargeBattery>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ChargingStationActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&ChargingStationActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&ChargingStationActionClient::result_callback, this, std::placeholders::_1);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<ChargeBattery>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(const GoalHandleCharge::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleCharge::SharedPtr,
        const std::shared_ptr<const ChargeBattery::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received feedback: Current battery level = %.1f%%", 
            feedback->current_battery_level);
    }

    void result_callback(const GoalHandleCharge::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), 
                    "Goal succeeded! Final battery level: %d%%", 
                    result.result->final_battery_level);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        rclcpp::shutdown();
    }
};

}  // namespace vacuum_robot

RCLCPP_COMPONENTS_REGISTER_NODE(vacuum_robot::ChargingStationActionClient)