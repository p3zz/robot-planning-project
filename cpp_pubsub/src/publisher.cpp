#include <chrono>
#include <functional>
#include <memory>
#include <math.h>
#include <cstdlib>
#include <vector>
#include <thread>
#include <iostream>
#include <fstream>

#include "dubins/dubins.hpp"
#include "map/map.hpp"
#include "decisions/decisions.hpp"

using namespace std;


int main()
{
  init_seed(0);
  cout << "The seed is " << get_seed() << endl;
  DubinPoint pursuer(2,8,M_PI*1.75);
  DubinPoint evader(8,2,M_PI*0.75);
  
  // Construct random room
  srand(get_seed());
  Polygon dim_room({ Point2D(0,0), Point2D(10,0), Point2D(10,10), Point2D(0,10) });
  Room room(dim_room);
  random_obstacles_side(&room, 4, 200);
  room.add_exit(Point2D(0,2));
  room.add_exit(Point2D(8,10));

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
    cerr<<"General error roadmap"<<endl;
    return 1;
  }
  
  //Construct Payoff Matrix
  PayoffMatrix mat(map);
  Path p, e;
  if(mat.computeMove(pursuer, evader, p, e))
  {
    ofstream myfile;
    myfile.open("moves.json", std::ofstream::trunc);
    myfile << get_path_json(p, e, 0.05);
    myfile.close();
  }
  else
  {
    cerr<<"General error matrix"<<endl;
    return 2;
  }
  
  return 0;
}