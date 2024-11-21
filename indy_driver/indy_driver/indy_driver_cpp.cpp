#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "neuromeka_cpp/indydcp3.h"

class IndyDriverCpp : public rclcpp::Node
{
public:
    IndyDriverCpp()
        : Node("indy_driver_cpp"), indy_("192.168.1.10") // Set IP of the robot
    {
        // Publish joint states
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        // Timer to publish
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            std::bind(&IndyDriverCpp::publish_joint_states, this)
        );

        RCLCPP_INFO(this->get_logger(), "IndyDriverCpp node has started.");
    }

private:
    void publish_joint_states()
    {
        // Get robot data
        Nrmk::IndyFramework::ControlData control_data;
        bool is_success = indy_.get_robot_data(control_data);

        if (is_success) // then publish joint states
        {
            auto joint_state_msg = sensor_msgs::msg::JointState();
            joint_state_msg.header.stamp = this->get_clock()->now();

            joint_state_msg.name = {"joint0", "joint1", "joint2", "joint3", "joint4", "joint5"};
            
            joint_state_msg.position.resize(control_data.q_size());
            joint_state_msg.velocity.resize(control_data.qdot_size());
            
            for (int i = 0; i < control_data.q_size(); i++)
            {
                joint_state_msg.position[i] = control_data.q(i) * M_PI / 180.0; 
            }

            for (int i = 0; i < control_data.qdot_size(); i++)
            {
                joint_state_msg.velocity[i] = control_data.qdot(i) * M_PI / 180.0;
            }

            joint_state_pub_->publish(joint_state_msg);
            // RCLCPP_INFO(this->get_logger(), "Published joint states.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to get robot data.");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    IndyDCP3 indy_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IndyDriverCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
