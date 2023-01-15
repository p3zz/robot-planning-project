#include "shelfino/shelfino.hpp"

PoseSubscriber::PoseSubscriber(std::optional<DubinPoint>& pose, std::string topic) : Node(topic), pose{pose} {
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    topic, 10, std::bind(&PoseSubscriber::topic_callback, this, _1));
}

void PoseSubscriber::topic_callback(const geometry_msgs::msg::Pose& msg) {
    tf2::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose.emplace(DubinPoint(msg.position.x, msg.position.y, yaw));
}

PoseListener::PoseListener(std::optional<DubinPoint>& pose): Node("shelfino2_listener"), pose{pose}{
    target_frame_ = this->declare_parameter<std::string>("target_frame", "shelfino2/base_link");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(1s, std::bind(&PoseListener::timer_callback, this));
}

void PoseListener::timer_callback() {
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "map";

    geometry_msgs::msg::TransformStamped t;

    try {
        t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        return;
    }

    tf2::Quaternion q(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose.emplace(DubinPoint(t.transform.translation.x, t.transform.translation.y, yaw));
}
