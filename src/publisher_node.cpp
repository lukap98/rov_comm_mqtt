#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mqtt/async_client.h>  
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("publisher_node"),
                      mqtt_client_("tcp://192.168.2.100:1883", "ros2_publisher"),
                      is_connected_(false)
    {
        // ROS2 Publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("ros2/mqtt/topic", 10);

        // Last will and testament msg
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);
        connOpts.set_will(mqtt::message("ros2/mqtt/topic", "ROS2 disconnected!", 1, false));;

        // MQTT Setup
        connect_to_broker();

        // Timer to publish messages every 3 seconds
        timer_ = this->create_wall_timer(3s, std::bind(&PublisherNode::publish_message, this));
    }

    ~PublisherNode()
    {
        if (mqtt_client_.is_connected())
        {
            mqtt::message_ptr disconnect_msg = mqtt::make_message("ros2/mqtt/topic", "ROS2 disconnected!");
            disconnect_msg->set_qos(1);
            mqtt_client_.publish(disconnect_msg)->wait();

            RCLCPP_INFO(this->get_logger(), "Sent disconnect message to MQTT broker.");
            mqtt_client_.disconnect()->wait();
        }
    }

private:
    void connect_to_broker()
    {
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);
        connOpts.set_will(mqtt::message("ros2/mqtt/topic", "ROS2 disconnected!", 1, false));;


        while (!is_connected_)
        {
            try
            {
                mqtt_client_.connect(connOpts)->wait();
                is_connected_ = true;
                RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker.");
            }
            catch (const mqtt::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker: %s", e.what());
                std::this_thread::sleep_for(5s); // Wait and retry
            }
        }
    }

    void publish_message()
    {
        if (!mqtt_client_.is_connected())
        {
            RCLCPP_WARN(this->get_logger(), "MQTT broker disconnected! Attempting to reconnect...");
            is_connected_ = false;
            connect_to_broker(); // Attempt to reconnect
        }

        // ROS2 Message
        auto message = std_msgs::msg::String();
        message.data = "ROS data";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        // Publish to ROS2 Topic
        publisher_->publish(message);

        // Publish to MQTT Topic
        mqtt::message_ptr mqtt_msg = mqtt::make_message("ros2/mqtt/topic", message.data);
        mqtt_msg->set_qos(1);

        try
        {
            mqtt_client_.publish(mqtt_msg)->wait();
            RCLCPP_INFO(this->get_logger(), "Published to MQTT topic 'ros2/mqtt/topic'");
        }
        catch (const mqtt::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to publish to MQTT: %s", e.what());
            is_connected_ = false; // Mark as disconnected
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    mqtt::async_client mqtt_client_;
    bool is_connected_;
};

// ✅ `main()` must be **after** the `PublisherNode` class
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}

