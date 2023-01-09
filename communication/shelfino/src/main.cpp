#include "shelfino/shelfino.hpp"

void thread_body(ShelfinoDto& dto){
    rclcpp::executors::SingleThreadedExecutor executor;
    auto gate_subscriber = std::make_shared<GatesSubscriber>(dto.gate_position);
    auto walls_subscriber = std::make_shared<WallsSubscriber>(dto.map_borders);
    executor.add_node(gate_subscriber);
    executor.add_node(walls_subscriber);
    std::cout<<"spinning"<<std::endl;
    executor.spin();
    rclcpp::shutdown();
}

// command to test subscriber thread
// ros2 topic pub --once gate_position geometry_msgs/msg/Pose '{orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: 1, y: 2, z: 0}}'
// ros2 topic pub --once map_borders geometry_msgs/msg/Polygon '{points:[{x: 0, y: 0, z: 0}, {x: 1, y: 0, z: 0}, {x: 1, y: 1, z: 0}, {x: 0, y: 1, z: 0}]}'
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto dto = ShelfinoDto();
    std::thread t(thread_body, std::ref(dto));
    t.detach();
    while(!dto.gate_position.is_valid() || !dto.map_borders.is_valid()){}
    std::cout<<dto.gate_position.get()<<std::endl;
    std::cout<<dto.map_borders.get().get_size()<<std::endl;
    return 0;
}