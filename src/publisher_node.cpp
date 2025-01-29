#include "rclcpp/rclcpp.hpp" // ROS2 Client Library for C++
#include "std_msgs/msg/string.hpp"
#include <mqtt/async_client.h>  // Paho MQTT library
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("publisher_node"),
                      mqtt_client_("tcp://192.168.2.100:1883", "ros2_publisher")  // Manually set for my local PC
                                                                                 // change to your broker's IP
    {
        // ROS2 Publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("ping_topic", 10);

        // MQTT Setup
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);

        try
        {
            mqtt_client_.connect(connOpts)->wait();
            RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker.");
        }
        catch (const mqtt::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker: %s", e.what());
        }

        // Timer to publish messages every 3 seconds
        timer_ = this->create_wall_timer(3s, std::bind(&PublisherNode::publish_message, this));
    }

    ~PublisherNode()
    {
        if (mqtt_client_.is_connected())
        {
            mqtt_client_.disconnect()->wait();
        }
    }

private:
    void publish_message()
    {
        // ROS2 Message
        auto message = std_msgs::msg::String();
        message.data = "Pinging from ROS over MQTT";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        // Publish to ROS2 Topic
        publisher_->publish(message);

        // Publish to MQTT Topic
        mqtt::message_ptr mqtt_msg = mqtt::make_message("ping_topic", message.data);
        mqtt_msg->set_qos(1); // Set Quality of Service level 1
        try
        {
            mqtt_client_.publish(mqtt_msg)->wait();
            RCLCPP_INFO(this->get_logger(), "Published to MQTT topic 'ping_topic'");
        }
        catch (const mqtt::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to publish to MQTT: %s", e.what());
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    mqtt::async_client mqtt_client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}