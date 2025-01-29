#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("publisher_node")
    {
        // Create a publisher to publish messages on the "ping_topic"
        publisher_ = this->create_publisher<std_msgs::msg::String>("ping_topic", 10);

        // Create a timer to publish messages periodically
        timer_ = this->create_wall_timer(2s, std::bind(&PublisherNode::publish_message, this));
    }

private:
    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = "Pinging from ROS";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}
