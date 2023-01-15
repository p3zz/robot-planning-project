#include "shelfino/shelfino.hpp"

void subscriber_body(ShelfinoDto& dto){
    rclcpp::executors::SingleThreadedExecutor executor;
    auto gate_subscriber = std::make_shared<GatesSubscriber>(dto.gates_position);
    auto walls_subscriber = std::make_shared<WallsSubscriber>(dto.map_borders);
    auto obstacles_subscriber = std::make_shared<ObstaclesSubscriber>(dto.obstacles);
    auto evader_pose_subscriber = std::make_shared<PoseSubscriber>(dto.evader_pose, "shelfino1_node");
    auto pursuer_pose_subscriber = std::make_shared<PoseSubscriber>(dto.pursuer_pose, "shelfino2_node");
    executor.add_node(gate_subscriber);
    executor.add_node(walls_subscriber);
    executor.add_node(obstacles_subscriber);
    executor.add_node(evader_pose_subscriber);
    executor.add_node(pursuer_pose_subscriber);
    std::cout<<"subscriber"<<std::endl;
    executor.spin();
    rclcpp::shutdown();
}

void publisher_body(ShelfinoDto& dto){
    auto roadmap_publisher = std::make_shared<RoadmapPublisher>(dto.roadmap);
    std::cout<<"publisher"<<std::endl;
    rclcpp::spin(roadmap_publisher);
    rclcpp::shutdown();
}

void service_body(ShelfinoDto& dto){
    rclcpp::executors::SingleThreadedExecutor executor;
    auto follow_path_client = std::make_shared<FollowPathClient>(dto.roadmap, dto.path_to_follow, dto.evader_pose, dto.pursuer_pose);
    executor.add_node(follow_path_client);
    std::cout<<"service"<<std::endl;
    executor.spin();
    rclcpp::shutdown();
}
// command to test subscriber thread
// ros2 topic pub --once gate_position geometry_msgs/msg/PoseArray '{header:{stamp:{sec: 0, nanosec: 0}}, poses:[{orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: 0, y: 2, z: 0}}, {orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: 8, y: 10, z: 0}}]}'
// ros2 topic pub --once map_borders geometry_msgs/msg/Polygon '{points:[{x: 0, y: 0, z: 0}, {x: 0, y: 10, z: 0}, {x: 10, y: 10, z: 0}, {x: 10, y: 0, z: 0}]}'
// ros2 topic pub --once obstacles custom_msgs/msg/ObstacleArrayMsg '{header:{stamp:{sec: 0, nanosec: 0}}, obstacles: [{header:{stamp:{sec: 0, nanosec: 0}}, polygon:{points:[{x: 3, y: 1, z: 0}, {x: 3, y: 2, z: 0}, {x: 4, y: 2, z: 0}, {x: 4, y: 1, z: 0}]}, radius: 0}, {header:{stamp:{sec: 0, nanosec: 0}}, polygon:{points:[{x: 8, y: 6, z: 0}, {x: 8, y: 7, z: 0}, {x: 9, y: 7, z: 0}, {x: 9, y: 6, z: 0}]}, radius: 0}]}'

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto dto = ShelfinoDto();
    std::thread subscriber(subscriber_body, std::ref(dto));
    subscriber.detach();
    // wait for map borders, gate position and obstacles
    while(!dto.gates_position.has_value() || !dto.map_borders.has_value() || !dto.obstacles.has_value()){}
    // setup room
    Room room(dto.map_borders.value());
    // setup obstacles
    for(auto &obstacle: dto.obstacles.value()){
        room.add_obstacle(obstacle);
    }
    // setup gates
    for(auto &gate: dto.gates_position.value()){
        room.add_exit(gate);
    }
    // build roadmap
    RoadMap rm(room);
    // TODO replace pursuer and evader position with the real one given by /tf
    DubinPoint pursuer(1, 1, M_PI*1.75);
    DubinPoint evader(7, 1, M_PI*0.75);

    if(!rm.construct_roadmap(60, 4, 0.5, 500, pursuer.get_point(), evader.get_point())){
        return 1;
    }
    dto.roadmap.emplace(rm);
    // // launch roadmap publisher
    std::thread publisher(publisher_body, std::ref(dto));
    publisher.detach();

    // // compute move
    // // TODO the computation of the next move must be done by the follow_path client 
    PayoffMatrix mat(rm);
    Path p, e;
    if(!mat.compute_move(pursuer, evader, p, e)){
        return 1;
    }
    dto.path_to_follow.emplace(p.l1.get_curve());
    // launch roadmap publisher
    std::thread service(service_body, std::ref(dto));
    service.join();
    return 0;
}