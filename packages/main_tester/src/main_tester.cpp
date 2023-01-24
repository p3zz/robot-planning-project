#include <cstdlib>
#include <iostream>
#include <fstream>

#include "dubins/dubins.hpp"
#include "map/map.hpp"
#include "planner/planner.hpp"

using namespace std;


int main()
{
  // Seed::init_seed(1920815646);
  Logger(Logger::INFO, "The seed is " + to_string(Seed::get_seed()));
  DubinPoint pursuer(-4.5, 4.5,  M_PI*1.75);
  DubinPoint evader(4.5, -4.5, M_PI*0.75);
  ROSTimer mytimer;
  
  // Construct random room
  // srand(Seed::get_seed());
  Polygon dim_room({ Point2D(-5, -5), Point2D(-5,5), Point2D(5,5), Point2D(5,-5) });
  Room room(dim_room);
  room.add_obstacle(Polygon({Point2D(-1, 0), Point2D(-1, 2), Point2D(0, 2), Point2D(0, 0)}));
  room.add_obstacle(Polygon({Point2D(3, -1), Point2D(4, -2), Point2D(2, -2)}));
  room.add_obstacle(Polygon({Point2D(-2, -3), Point2D(-2, -2), Point2D(-1, -2), Point2D(-1, -3)}));
  room.add_exit(Point2D(-5, -4.5));
  room.add_exit(Point2D(5, 4.5));
  Logger(Logger::INFO, "Time passed to construct room: " + mytimer);
  mytimer.rst();

  //Construct roadmap
  RoadMap map(room);
  if(!map.construct_roadmap(60, 4, 0.5, 500, pursuer.get_point(), evader.get_point())){
    Logger(Logger::ERROR, "General error roadmap");
    return 1;
  }

  ofstream map_file;
  map_file.open ("map.json", std::ofstream::trunc);
  map_file << map.get_json();
  map_file.close();

  Logger(Logger::INFO, "Time passed to construct roadmap: " + mytimer);
  mytimer.rst();
  
  //Construct Payoff Matrix
  PayoffMatrix mat(map);

  Path p, e;
  auto p_last_src = pursuer;
  auto e_last_src = evader;
  std::vector<DubinLink> p_moves;
  std::vector<DubinLink> e_moves;
  const int max_moves = 50;
  int n_moves = 0;
  // while moves exists
  while (mat.compute_move(p_last_src, e_last_src, p, e) && n_moves < max_moves){
    p_last_src = p.l1.get_dst();
    e_last_src = e.l1.get_dst();
    p_moves.push_back(p.l1);
    e_moves.push_back(e.l1);
    bool same_destination = p.l1.get_dst() == e.l1.get_dst();

    bool idle_while_catching = p.l1.get_src() == e.l1.get_dst();
    
    bool evaded = false;
    for(int i=0;i<room.get_num_exits();i++){
      auto exit = room.get_exit(i, true);
      if(e.l1.get_dst().get_point() == exit)
      {
        evaded=true;
        break;
      }
      if(e.l2.get_dst().get_point() == exit)
      {
        same_destination = p.l2.get_dst() == e.l2.get_dst();
        idle_while_catching = p.l2.get_src() == e.l2.get_dst();
        evaded=!(same_destination || idle_while_catching);
        p_moves.push_back(p.l2);
        e_moves.push_back(e.l2);
        break;
      }
    }

    if(evaded) Logger(Logger::INFO, "Evader has reached the exit");
    if(same_destination) Logger(Logger::INFO, "Evader and pursuer have computed the same destination");
    if(idle_while_catching) Logger(Logger::INFO, "Evader has computed the position of pursuer as destination");

    if(evaded || same_destination || idle_while_catching){
      break;
    }

    Logger(Logger::INFO, "["+to_string(n_moves++)+"] Moves found");
  }
  
  ofstream moves_file;
  moves_file.open("moves.json", std::ofstream::trunc);

  moves_file << get_pursuer_evader_moves_json(p_moves, e_moves, 0.05);

  moves_file.close();

  // Logger(Logger::INFO, "Time passed to ompute matrix: " + mytimer);
  // mytimer.rst();
  
  return 0;
}