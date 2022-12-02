#ifndef PATH_SOLVER_H
#define PATH_SOLVER_H

#include "utils/utils.h"

#define MAX_VERTEXES 10
#define MAX_OBSTACLES 10

class Point2D
{
    public:
        double x, y;
        Point2D(double x, double y):x{x},y{y}{}
}

class Obstacle
{
    private:
        int num_v=0;
    public:
        Point2D vertexes[MAX_VERTEXES];
        Obstacle(){}
        bool addVertex(Point2D v)
        {
            if(num_v==MAX_VERTEXES)
                return false;
            vertexes[num_v++]=v;
            return true;
        }
        int getNumVertexes(){return num_v;}

}

class Room
{
    private: 
        double h;
        double w;
        int num_o=0;
        
    public:
        Obstacle obstacles[MAX_OBSTACLES];
        Room(double h, double w):h{h},w{w}{}
        bool addObstacle(Obstacle o)
        {
            if(num_o==MAX_OBSTACLES)
                return false;
            obstacles[num_o++]=o;
            return true;
        }
        int getNumObstacles(){return num_o;}
        int getHeight(){return h;}
        int getWidth(){return w;}

};

class PathSolver
{
    private: 
        Room r;
        int points;
        double t_max;
        len_path=0; //in points

    public:
        PathSolver(Room r, int points, double t_max):r{r},points{points},t_max{t_max}{} //time_max in seconds
        Point2D* constructPath(Point2D robot_pos, Point2D goal_pos);
        int getLenPath(){return len_path;}
}

#endif