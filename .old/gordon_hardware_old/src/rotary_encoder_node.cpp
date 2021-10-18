#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "gordon_hardware/rotary_encoder.hpp"

using namespace std::chrono_literals;

class RotaryEncoderNode : public rclcpp::Node {
public:
    RotaryEncoderNode()
        : Node("rotary_encoder")
    {
        auto a_pin = this->declare_parameter<int>("a_pin", 0);
        auto b_pin = this->declare_parameter<int>("b_pin", 0);

        publisher = this->create_publisher<std_msgs::msg::Int32>("count", 10);

        timer = this->create_wall_timer(20ms, std::bind(&RotaryEncoderNode::timer_callback, this));

        rotary_encoder = new RotaryEncoder(a_pin, b_pin);
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::Int32();
        message.data = rotary_encoder->count;

        publisher->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;

    RotaryEncoder* rotary_encoder;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RotaryEncoderNode>());
    rclcpp::shutdown();
    return 0;
}
