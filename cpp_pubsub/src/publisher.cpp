#include <cstdlib>
#include <iostream>
#include <fstream>

#include "dubins/dubins.hpp"
#include "map/map.hpp"
#include "decisions/decisions.hpp"

using namespace std;


int main()
{
  Seed::init_seed(1920815646);
  Logger(Logger::INFO, "The seed is " + to_string(Seed::get_seed()));
  DubinPoint pursuer(7,10,M_PI*1.75);
  DubinPoint evader(13,10,M_PI*0.75);
  ROSTimer mytimer;
  
  // Construct random room
  srand(Seed::get_seed());
  Polygon dim_room({ Point2D(10,5), Point2D(5,10), Point2D(10,15), Point2D(15,10) });
  Room room(dim_room);
  random_obstacles_side(&room, 2, 100);
  room.add_exit(Point2D(7,8));
  room.add_exit(Point2D(7,12));

  Logger(Logger::INFO, "Time passed to construct room: " + mytimer);
  mytimer.rst();

  //Construct roadmap
  RoadMap map(room);
  if(map.construct_roadmap(60, 4, 0.5, 500, pursuer.get_point(), evader.get_point())) //knn=4 is the best choice (up, down, left and right in the ideal case)
  {
    ofstream myfile;
    myfile.open ("map.json", std::ofstream::trunc);
    myfile << map.get_json();
    myfile.close();
  }
  else
  {
    Logger(Logger::ERROR, "General error roadmap");
    return 1;
  }

  Logger(Logger::INFO, "Time passed to construct roadmap: " + mytimer);
  mytimer.rst();
  
  //Construct Payoff Matrix
  PayoffMatrix mat(map);
  Path p, e;
  if(mat.compute_move(pursuer, evader, p, e))
  {
    ofstream myfile;
    myfile.open("moves.json", std::ofstream::trunc);
    myfile << get_path_json(p, e, 0.05);
    myfile.close();
  }
  else
  {
    Logger(Logger::ERROR, "General error matrix");
    return 2;
  }

  Logger(Logger::INFO, "Time passed to ompute matrix: " + mytimer);
  mytimer.rst();
  
  return 0;
}