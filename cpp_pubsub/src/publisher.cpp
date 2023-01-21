#include <cstdlib>
#include <iostream>
#include <fstream>

#include "dubins/dubins.hpp"
#include "map/map.hpp"
#include "decisions/decisions.hpp"

using namespace std;


int main()
{
  // Seed::init_seed(1920815646);
  // Logger(Logger::INFO, "The seed is " + to_string(Seed::get_seed()));
  DubinPoint pursuer(3, 1, M_PI*1.75);
  DubinPoint evader(1, 3, M_PI*0.75);
  ROSTimer mytimer;
  
  // Construct random room
  // srand(Seed::get_seed());
  Polygon dim_room({ Point2D(-5, -5), Point2D(-5,5), Point2D(5,5), Point2D(5,-5) });
  Room room(dim_room);
  room.add_obstacle(Polygon({Point2D(-1, 0), Point2D(-1, 1), Point2D(0, 1), Point2D(0, 0)}));
  room.add_obstacle(Polygon({Point2D(3, 0), Point2D(4, -1), Point2D(2, -1)}));
  room.add_exit(Point2D(-5,-2));
  room.add_exit(Point2D(-2,-5));

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

  // while moves exists and both shelfinos don't compute the same destination etc..
  while (mat.compute_move(p_last_src, e_last_src, p, e) && !(p.l1.get_dst() == e.l1.get_dst()) && !(p.l1.get_src() == e.l1.get_dst())){
    p_last_src = p.l1.get_dst();
    e_last_src = e.l1.get_dst();
    p_moves.push_back(p.l1);
    e_moves.push_back(e.l1);
    Logger(Logger::INFO, "Moves found");
  }
  
  ofstream moves_file;
  moves_file.open("moves.json", std::ofstream::trunc);

  moves_file << get_pursuer_evader_moves_json(p_moves, e_moves, 0.05);

  moves_file.close();

  // Logger(Logger::INFO, "Time passed to ompute matrix: " + mytimer);
  // mytimer.rst();
  
  return 0;
}