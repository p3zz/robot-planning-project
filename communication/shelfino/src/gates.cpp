#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;


class GatesSubscriber : public rclcpp::Node{
    public:
    GatesSubscriber() : Node("gates_subscriber") {
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "gate_position", 10, std::bind(&GatesSubscriber::topic_callback, this, _1));
    }

    private:
    void topic_callback(const geometry_msgs::msg::Pose & msg) const {
        // RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << "'");
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};
