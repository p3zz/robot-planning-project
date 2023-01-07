#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;


class WallsSubscriber : public rclcpp::Node{
    public:
    WallsSubscriber() : Node("walls_subscriber") {
        subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
        "map_borders", 10, std::bind(&WallsSubscriber::topic_callback, this, _1));
    }

    private:
    void topic_callback(const geometry_msgs::msg::Polygon & msg) const {
        // RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << "'");
    }
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_;
};
