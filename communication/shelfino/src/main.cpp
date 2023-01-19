#include "shelfino/shelfino.hpp"

const std::string evader = "shelfino1";
const std::string pursuer = "shelfino2";

void subscriber_body(ShelfinoDto& dto){
    rclcpp::executors::SingleThreadedExecutor executor;
    auto gate_subscriber = std::make_shared<GatesSubscriber>(dto.gates_position);
    auto walls_subscriber = std::make_shared<WallsSubscriber>(dto.map_borders);
    auto obstacles_subscriber = std::make_shared<ObstaclesSubscriber>(dto.obstacles);
    auto evader_pose_subscriber = std::make_shared<PoseSubscriber>(dto.evader_pose, evader + "/transform", evader + "_transform");
    auto pursuer_pose_subscriber = std::make_shared<PoseSubscriber>(dto.pursuer_pose, pursuer + "/transform", pursuer + "_transform");
    executor.add_node(gate_subscriber);
    executor.add_node(walls_subscriber);
    executor.add_node(obstacles_subscriber);
    executor.add_node(evader_pose_subscriber);
    executor.add_node(pursuer_pose_subscriber);
    executor.spin();
    rclcpp::shutdown();
}

void publisher_body(ShelfinoDto& dto){
    rclcpp::executors::SingleThreadedExecutor executor;
    auto roadmap_publisher = std::make_shared<RoadmapPublisher>(dto.roadmap);
    auto path_evader_publisher = std::make_shared<PathPublisher>(dto.evader_path_to_follow, evader + "/plan", evader + "_plan");
    auto path_pursuer_publisher = std::make_shared<PathPublisher>(dto.pursuer_path_to_follow, pursuer + "/plan", pursuer + "_plan");
    executor.add_node(roadmap_publisher);
    executor.add_node(path_evader_publisher);
    executor.add_node(path_pursuer_publisher);
    executor.spin();
    rclcpp::shutdown();
}

void service_body(ShelfinoDto& dto){
    rclcpp::executors::SingleThreadedExecutor executor;
    auto follow_path_evader_client = std::make_shared<FollowPathClient>(dto.roadmap, dto.evader_path_to_follow, dto.evader_pose,
        dto.pursuer_pose, Shelfino::Evader, evader + "/follow_path", evader + "_follow_path");
    auto follow_path_pursuer_client = std::make_shared<FollowPathClient>(dto.roadmap, dto.pursuer_path_to_follow, dto.evader_pose,
        dto.pursuer_pose, Shelfino::Pursuer, pursuer + "/follow_path", pursuer + "_follow_path");
    executor.add_node(follow_path_evader_client);
    executor.add_node(follow_path_pursuer_client);
    executor.spin();
    rclcpp::shutdown();
}
// command to test subscriber thread
// ros2 topic pub --once gate_position geometry_msgs/msg/PoseArray '{header:{stamp:{sec: 0, nanosec: 0}}, poses:[{orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: -10, y: -5, z: 0}}, {orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: 10, y: 5, z: 0}}]}'
// ros2 topic pub --once map_borders geometry_msgs/msg/Polygon '{points:[{x: -10, y: -10, z: 0}, {x: -10, y: 10, z: 0}, {x: 10, y: 10, z: 0}, {x: 10, y: -10, z: 0}]}'
// ros2 topic pub --once obstacles custom_msgs/msg/ObstacleArrayMsg '{header:{stamp:{sec: 0, nanosec: 0}}, obstacles: [{header:{stamp:{sec: 0, nanosec: 0}}, polygon:{points:[{x: 3, y: 1, z: 0}, {x: 3, y: 2, z: 0}, {x: 4, y: 2, z: 0}, {x: 4, y: 1, z: 0}]}, radius: 0}, {header:{stamp:{sec: 0, nanosec: 0}}, polygon:{points:[{x: 8, y: 6, z: 0}, {x: 8, y: 7, z: 0}, {x: 9, y: 7, z: 0}, {x: 9, y: 6, z: 0}]}, radius: 0}]}'
// ros2 topic pub --once pursuer_pose geometry_msgs/msg/Pose '{orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: 0, y: 0, z: 0}}'
// ros2 topic pub --once evader_pose geometry_msgs/msg/Pose '{orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: 0, y: 1, z: 0}}'

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

    while(!dto.pursuer_pose.has_value() || !dto.evader_pose.has_value()){}

    // build roadmap
    RoadMap rm(room);

    std::cout<<"Building roadmap"<<std::endl;

    // build roadmap
    if(!rm.construct_roadmap(60, 4, 0.5, 500, dto.pursuer_pose.value().get_point(), dto.evader_pose.value().get_point())){
        std::cout<<"Error while building roadmap"<<std::endl;
        return 1;
    }

    std::cout<<"Roadmap built"<<std::endl;

    std::ofstream myfile;
    myfile.open ("map.json", std::ofstream::trunc);
    myfile << rm.get_json();
    myfile.close();

    dto.roadmap.emplace(rm);
    
    // // launch roadmap publisher
    std::thread publisher(publisher_body, std::ref(dto));
    publisher.detach();

    std::thread service(service_body, std::ref(dto));
    service.join();
    return 0;
}