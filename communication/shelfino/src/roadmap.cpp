#include "shelfino/shelfino.hpp"

custom_msgs::msg::GeometryGraph msg_from_roadmap(RoadMap rm, std_msgs::msg::Header h){
  custom_msgs::msg::GeometryGraph gg;
  std::vector<custom_msgs::msg::Edges> nodes_edges;
  std::vector<geometry_msgs::msg::Point> nodes;
  for(auto &node: rm.get_nodes()){
    geometry_msgs::msg::Point p;
    p.x = node.x;
    p.y = node.y;
    p.z = 0;
    nodes.push_back(p);
    std::vector<unsigned int> neighbors;
    for(auto &link: rm.get_links()){
      int i = -1;
      if(node == link.node1){
        i = rm.get_node_index(link.node2);
      }
      else if(node == link.node2){
        i = rm.get_node_index(link.node1);
      }
      if(i > -1){
        neighbors.push_back(i);
      }
    }
    custom_msgs::msg::Edges edges;
    edges.node_ids = neighbors;
    nodes_edges.push_back(edges);
  }

  gg.header = h;
  gg.nodes = nodes;
  gg.edges = nodes_edges;

  return gg;
}

RoadmapPublisher::RoadmapPublisher(std::optional<RoadMap>& roadmap): Node("roadmap_publisher"), count_(0) {
    publisher_ = this->create_publisher<custom_msgs::msg::GeometryGraph>("roadmap", 10);
    if(!roadmap.has_value())return;
    std_msgs::msg::Header h;
    h.stamp = this->get_clock()->now();
    auto message = msg_from_roadmap(roadmap.value(), h);
    publisher_->publish(message);
    std::cout<<"roadmap sent"<<std::endl;
}