#include "shelfino/shelfino.hpp"

void subscriber_body(ShelfinoDto& dto){
    rclcpp::executors::SingleThreadedExecutor executor;
    auto gate_subscriber = std::make_shared<GatesSubscriber>(dto.gates_position);
    auto walls_subscriber = std::make_shared<WallsSubscriber>(dto.map_borders);
    auto obstacles_subscriber = std::make_shared<ObstaclesSubscriber>(dto.obstacles);
    auto pose_listener = std::make_shared<PoseListener>(dto.pose);
    auto follow_path_client = std::make_shared<FollowPathClient>(dto.path_to_follow);
    executor.add_node(gate_subscriber);
    executor.add_node(walls_subscriber);
    executor.add_node(obstacles_subscriber);
    executor.add_node(pose_listener);
    executor.add_node(follow_path_client);
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
// ros2 topic pub --once gate_position geometry_msgs/msg/PoseArray '{header:{stamp:{sec: 0, nanosec: 0}}, poses:[{orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: 0, y: 2, z: 0}}, {orientation:{x: 0, y: 0, z: 0, w: 1}, position:{x: 8, y: 10, z: 0}}]}'
// ros2 topic pub --once map_borders geometry_msgs/msg/Polygon '{points:[{x: 0, y: 0, z: 0}, {x: 0, y: 10, z: 0}, {x: 10, y: 10, z: 0}, {x: 10, y: 0, z: 0}]}'
// ros2 topic pub --once obstacles custom_msgs/msg/ObstacleArrayMsg '{header:{stamp:{sec: 0, nanosec: 0}}, obstacles: [{header:{stamp:{sec: 0, nanosec: 0}}, polygon:{points:[{x: 3, y: 1, z: 0}, {x: 3, y: 2, z: 0}, {x: 4, y: 2, z: 0}, {x: 4, y: 1, z: 0}]}, radius: 0}, {header:{stamp:{sec: 0, nanosec: 0}}, polygon:{points:[{x: 8, y: 6, z: 0}, {x: 8, y: 7, z: 0}, {x: 9, y: 7, z: 0}, {x: 9, y: 6, z: 0}]}, radius: 0}]}'

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto dto = ShelfinoDto();
    std::thread subscriber(subscriber_body, std::ref(dto));
    subscriber.detach();
    // wait for map borders, gate position and obstacles
    while(!dto.gates_position.is_valid() || !dto.map_borders.is_valid() || !dto.obstacles.is_valid()){}
    Room room(dto.map_borders.get());
    for(auto &obstacle: dto.obstacles.get()){
        room.add_obstacle(obstacle);
    }
    for(auto &gate: dto.gates_position.get()){
        room.add_exit(gate);
    }
    Seed::init_seed(0);
    std::cout << "Seed: " << Seed::get_seed() << std::endl;
    RoadMap rm(room);
    DubinPoint pursuer(1, 1, M_PI*1.75);
    DubinPoint evader(7, 1, M_PI*0.75);
    if(!rm.construct_roadmap(60, 4, 0.5, 500, pursuer.get_point(), evader.get_point())){
        return -1;
    }
    // write map
    std::ofstream myfile;
    myfile.open ("map.json", std::ofstream::trunc);
    myfile << rm.get_json();
    myfile.close();
    // launch publisher
    std::thread publisher(publisher_body, rm);
    publisher.detach();
    PayoffMatrix mat(rm);
    Path p, e;
    // compute move
    if(!mat.computeMove(pursuer, evader, p, e)){
        return -1;
    }
    if(!mat.computeMove(p.l1.get_dst(), e.l1.get_dst(), p, e)){
        return -1;
    }
    // write moves
    myfile.open("moves.json", std::ofstream::trunc);
    myfile << get_path_json(p, e, 0.05);
    myfile.close();
    dto.path_to_follow.set(p.l1.get_curve());
    return 0;
}