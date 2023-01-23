#include "navigation/navigation.hpp"

const std::string EVADER_NAMESPACE = "shelfino1";
const std::string PURSUER_NAMESPACE = "shelfino2";

void subscriber_body(EnvironmentDto& env, ShelfinoDto& evader, ShelfinoDto& pursuer){
    rclcpp::executors::SingleThreadedExecutor executor;
    auto gate_subscriber = std::make_shared<GatesSubscriber>(env.gates_position);
    auto walls_subscriber = std::make_shared<WallsSubscriber>(env.map_borders);
    auto obstacles_subscriber = std::make_shared<ObstaclesSubscriber>(env.obstacles);
    auto evader_pose_subscriber = std::make_shared<PoseSubscriber>(evader.pose, EVADER_NAMESPACE + "/transform", EVADER_NAMESPACE + "_transform");
    auto pursuer_pose_subscriber = std::make_shared<PoseSubscriber>(pursuer.pose, PURSUER_NAMESPACE + "/transform", PURSUER_NAMESPACE + "_transform");
    executor.add_node(gate_subscriber);
    executor.add_node(walls_subscriber);
    executor.add_node(obstacles_subscriber);
    executor.add_node(evader_pose_subscriber);
    executor.add_node(pursuer_pose_subscriber);
    executor.spin();
    rclcpp::shutdown();
}

void publisher_body(EnvironmentDto& env, ShelfinoDto& evader, ShelfinoDto& pursuer){
    rclcpp::executors::SingleThreadedExecutor executor;
    auto roadmap_publisher = std::make_shared<RoadmapPublisher>(env.roadmap);
    auto path_evader_publisher = std::make_shared<PathPublisher>(evader.path_to_follow, EVADER_NAMESPACE + "/plan", EVADER_NAMESPACE + "_plan");
    auto path_pursuer_publisher = std::make_shared<PathPublisher>(pursuer.path_to_follow, PURSUER_NAMESPACE + "/plan", PURSUER_NAMESPACE + "_plan");
    executor.add_node(roadmap_publisher);
    executor.add_node(path_evader_publisher);
    executor.add_node(path_pursuer_publisher);
    executor.spin();
    rclcpp::shutdown();
}

void service_body(EnvironmentDto& env, ShelfinoDto& evader, ShelfinoDto& pursuer){
    rclcpp::executors::SingleThreadedExecutor executor;
    auto follow_path_evader_client = std::make_shared<FollowPathClient>(env.roadmap, ShelfinoType::Evader, evader, pursuer, EVADER_NAMESPACE + "/follow_path", EVADER_NAMESPACE + "_follow_path");
    auto follow_path_pursuer_client = std::make_shared<FollowPathClient>(env.roadmap, ShelfinoType::Pursuer, evader, pursuer, PURSUER_NAMESPACE + "/follow_path", PURSUER_NAMESPACE + "_follow_path");
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
    auto env = EnvironmentDto();
    auto evader = ShelfinoDto();
    auto pursuer = ShelfinoDto();
    std::thread subscriber(subscriber_body, std::ref(env), std::ref(evader), std::ref(pursuer));
    subscriber.detach();
    // wait for map borders, gate position and obstacles
    while(!env.gates_position.has_value() || !env.map_borders.has_value() || !env.obstacles.has_value()){}
    // setup room
    Room room(env.map_borders.value());
    // setup obstacles
    for(auto &obstacle: env.obstacles.value()){
        room.add_obstacle(obstacle);
    }
    // setup gates
    for(auto &gate: env.gates_position.value()){
        room.add_exit(gate);
    }

    while(!pursuer.pose.has_value() || !evader.pose.has_value()){}

    // build roadmap
    RoadMap rm(room);

    std::cout<<"Building roadmap"<<std::endl;

    // build roadmap
    if(!rm.construct_roadmap(60, 4, 0.5, 500, pursuer.pose.value().get_point(), evader.pose.value().get_point())){
        std::cout<<"Error while building roadmap"<<std::endl;
        return 1;
    }

    std::cout<<"Roadmap built"<<std::endl;

    // std::ofstream myfile;
    // myfile.open ("map.json", std::ofstream::trunc);
    // myfile << rm.get_json();
    // myfile.close();

    env.roadmap.emplace(rm);
    
    // // launch roadmap publisher
    std::thread publisher(publisher_body, std::ref(env), std::ref(evader), std::ref(pursuer));
    publisher.detach();

    std::thread service(service_body, std::ref(env), std::ref(evader), std::ref(pursuer));
    service.join();
    return 0;
}