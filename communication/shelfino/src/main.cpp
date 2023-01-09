#include "shelfino/shelfino.hpp"

void thread_body(Point2D& gate_position){
    auto gate_subscriber = std::make_shared<GatesSubscriber>(gate_position);
    std::cout<<"spinning"<<std::endl;
    rclcpp::spin(gate_subscriber);
    rclcpp::shutdown();
}

// command to test subscriber thread
// ros2 topic pub --once gate_position geometry_msgs/msg/Pose '{orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: 1, y: 2, z: 0}}'
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    Point2D p(0,0);
    std::thread t(thread_body, std::ref(p));
    t.detach();
    while(1){
        std::cout<<p<<std::endl;
        sleep(1);
    }
    return 0;
}