#include "navigation/navigation.hpp"

PoseSubscriber::PoseSubscriber(std::optional<DubinPoint>& pose, std::string topic, std::string node_name) : Node(node_name), pose{pose} {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    topic, qos, std::bind(&PoseSubscriber::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Waiting");
}

void PoseSubscriber::topic_callback(const geometry_msgs::msg::TransformStamped& msg) {
    tf2::Quaternion q(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose.emplace(DubinPoint(msg.transform.translation.x, msg.transform.translation.y, yaw));
    // if(pose.has_value()){
    //     return;
    // }
    // RCLCPP_INFO(this->get_logger(), "Pose received: (x: %f, y: %f, th: %f)", pose.value().x, pose.value().y, pose.value().th);
}

// PoseListener::PoseListener(std::optional<DubinPoint>& pose): Node("shelfino2_listener"), pose{pose}{
//     target_frame_ = this->declare_parameter<std::string>("target_frame", "shelfino2/base_link");
//     tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//     timer_ = this->create_wall_timer(1s, std::bind(&PoseListener::timer_callback, this));
// }

// void PoseListener::timer_callback() {
//     std::string fromFrameRel = target_frame_.c_str();
//     std::string toFrameRel = "map";

//     geometry_msgs::msg::TransformStamped t;

//     try {
//         t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
//     } catch (const tf2::TransformException & ex) {
//         return;
//     }

//     tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
//     tf2::Matrix3x3 m(q);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);
//     pose.emplace(DubinPoint(t.transform.translation.x, t.transform.translation.y, yaw));
// }
