#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <fstream>
#include <vector>
#include <cmath>
#include <tf2/exceptions.h>
#include <chrono>

using namespace std::chrono_literals;

class VacuumTracker : public rclcpp::Node
{
public:
    VacuumTracker() : Node("vacuum_tracker")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            1000ms, 
            std::bind(&VacuumTracker::timer_callback, this));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vc_path", 10);

        RCLCPP_INFO(this->get_logger(), "Vacuum Tracker Node Started (10 Hz, No Filter)");
    }

    ~VacuumTracker()
    {
        save_to_csv();
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped vc_to_floor;
        geometry_msgs::msg::TransformStamped cs_to_floor;

        try
        {
            // Get transform from tag36h11:2 (VC) to tag36h11:0 (floor)
            vc_to_floor = tf_buffer_->lookupTransform("tag36h11:0", "tag36h11:2", 
                                           tf2::TimePointZero);

            // Get transform from tag36h11:1 (CS) to tag36h11:0 (floor)
            cs_to_floor = tf_buffer_->lookupTransform("tag36h11:0", "tag36h11:1", 
                                           tf2::TimePointZero);

            // Project VC position onto the floor plane (z=0)
            double vc_x = vc_to_floor.transform.translation.x;
            double vc_y = vc_to_floor.transform.translation.y;

            // Project CS position onto the floor plane (z=0)
            if (!cs_position_set_)
            {
                cs_x_ = cs_to_floor.transform.translation.x;
                cs_y_ = cs_to_floor.transform.translation.y;
                cs_position_set_ = true;
                RCLCPP_INFO(this->get_logger(), "CS position: (%.3f, %.3f)", cs_x_, cs_y_);
            }

            // Transform VC position relative to CS
            double vc_relative_x = vc_x - cs_x_;
            double vc_relative_y = vc_y - cs_y_;
            
            // Store the position
            vc_path_x_.push_back(vc_relative_x);
            vc_path_y_.push_back(vc_relative_y);
            timestamps_.push_back(vc_to_floor.header.stamp.sec + 
                                 vc_to_floor.header.stamp.nanosec * 1e-9);

            // Create and publish path for visualization
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = this->now();
            path_msg.header.frame_id = "tag36h11:1"; // Relative to CS

            for (size_t i = 0; i < vc_path_x_.size(); ++i)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "tag36h11:1";
                pose.pose.position.x = vc_path_x_[i];
                pose.pose.position.y = vc_path_y_[i];
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                path_msg.poses.push_back(pose);
            }

            path_pub_->publish(path_msg);

            if (vc_path_x_.size() % 100 == 0) 
            {
                RCLCPP_INFO(this->get_logger(), "Recorded %zu positions", vc_path_x_.size());
            }
        }
        catch (tf2::TransformException &ex)
        {
            // Silently continue, transforms may not be available yet
        }
    }

    void save_to_csv()
    {
        // Use original filename and original structure
        std::string filename = "vc_path.csv"; 
        std::ofstream file(filename);

        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return;
        }

        // ORIGINAL 3-COLUMN STRUCTURE
        file << "timestamp,x,y\n"; 
        
        // Use raw timestamp for plotting script compatibility (as in original code)
        for (size_t i = 0; i < vc_path_x_.size(); ++i)
        {
            file << timestamps_[i] << "," 
                 << vc_path_x_[i] << "," 
                 << vc_path_y_[i] << "\n";
        }

        file.close();
        RCLCPP_INFO(this->get_logger(), "Saved %zu positions to %s", 
                    vc_path_x_.size(), filename.c_str());
        
        if (!vc_path_x_.empty()) {
            double final_x = vc_path_x_.back();
            double final_y = vc_path_y_.back();
            RCLCPP_INFO(this->get_logger(), "Final VC position relative to CS: (%.3f, %.3f)", 
                        final_x, final_y);
            
            double distance = std::sqrt(final_x * final_x + final_y * final_y);
            RCLCPP_INFO(this->get_logger(), "Distance from CS: %.3f meters", distance);
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    std::vector<double> vc_path_x_;
    std::vector<double> vc_path_y_;
    std::vector<double> timestamps_;
    
    double cs_x_;
    double cs_y_;
    bool cs_position_set_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VacuumTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}