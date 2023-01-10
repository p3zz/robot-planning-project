#include "shelfino/shelfino.hpp"

void subscriber_body(ShelfinoDto& dto){
    rclcpp::executors::SingleThreadedExecutor executor;
    auto gate_subscriber = std::make_shared<GatesSubscriber>(dto.gate_position);
    auto walls_subscriber = std::make_shared<WallsSubscriber>(dto.map_borders);
    auto obstacles_subscriber = std::make_shared<ObstaclesSubscriber>(dto.obstacles);
    auto pose_listener = std::make_shared<PoseListener>(dto.pose);
    executor.add_node(gate_subscriber);
    executor.add_node(walls_subscriber);
    executor.add_node(obstacles_subscriber);
    executor.add_node(pose_listener);
    std::cout<<"spinning"<<std::endl;
    executor.spin();
    rclcpp::shutdown();
}

void publisher_body(RoadMap rm){
    auto roadmap_publisher = std::make_shared<RoadmapPublisher>(rm);
    rclcpp::spin(roadmap_publisher);
    rclcpp::shutdown();
}

// command to test subscriber thread
// ros2 topic pub --once gate_position geometry_msgs/msg/Pose '{orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: 1, y: 2, z: 0}}'
// ros2 topic pub --once map_borders geometry_msgs/msg/Polygon '{points:[{x: 0, y: 0, z: 0}, {x: 1, y: 0, z: 0}, {x: 1, y: 1, z: 0}, {x: 0, y: 1, z: 0}]}'
// ros2 topic pub --once obstacles custom_msgs/msg/ObstacleArrayMsg '{header:{stamp:{sec: 0, nanosec: 0}}, obstacles: [{header:{stamp:{sec: 0, nanosec: 0}}, polygon:{points:[{x: 0, y: 0, z: 0}, {x: 1, y: 0, z: 0}, {x: 1, y: 1, z: 0}, {x: 0, y: 1, z: 0}]}, radius: 0}, {header:{stamp:{sec: 0, nanosec: 0}}, polygon:{points:[{x: 2, y: 1, z: 0}, {x: 1, y: 0, z: 0}, {x: 3, y: 2, z: 0}, {x: 0, y: 1, z: 0}]}, radius: 0}]}'

int main(int argc, char * argv[]) {
    init_seed(0);
    std::cout << "The seed is " << get_seed() << std::endl;
    rclcpp::init(argc, argv);
    auto dto = ShelfinoDto();
    std::thread subscriber(subscriber_body, std::ref(dto));
    subscriber.detach();
    // wait for map borders, gate position and obstacles
    while(!dto.gate_position.is_valid() || !dto.map_borders.is_valid() || !dto.obstacles.is_valid()){}
    std::cout<<"Gate: "<<dto.gate_position.get()<<std::endl;
    std::cout<<"Map borders: "<<dto.map_borders.get().get_size()<<std::endl;
    std::cout<<"Obstacles: "<<dto.obstacles.get().size()<<std::endl;
    Room room(Polygon({ Point2D(0,0), Point2D(10,0), Point2D(10,10), Point2D(0,10) }));
    for(auto obstacle: dto.obstacles.get()){
        room.add_obstacle(obstacle);
    }
    RoadMap rm(room);
    rm.construct_roadmap(60, 4, 0.5, 500);
    std::thread publisher(publisher_body, rm);
    publisher.detach();
    std::cout<<"Nodes: "<<rm.get_nodes().size()<<std::endl;
    std::cout<<"Links: "<<rm.get_links().size()<<std::endl;
    return 0;
}