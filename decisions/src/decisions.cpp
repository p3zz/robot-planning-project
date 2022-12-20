#include "decisions/decisions.hpp"
using namespace std;

double min(double n1, double n2)
{
    return n2>n1? n2:n1;
}

Point2D getNearestNode(Point2D position, std::vector<Point2D> nodes)
{
    Point2D p;
    double dist_min=POINT_COORD_MAX, dist;
    for(int i=0; i<(int)nodes.size(); i++)
    {
        dist=distance(position, nodes.at(i));
        if(dist<dist_min)
        {
            dist_min=dist;
            p=nodes.at(i);
        }
    }
    return p;
}

double score(Point2D pursuer, Point2D evader, Room& r)
{
    double max_distance=sqrt(pow(r.getHeight(),2)+pow(r.getWidth(),2));
    double min_exit=POINT_COORD_MAX;
    for(int i=0; i<r.getNumExits(); i++)
        min_exit=min(min_exit, distance(evader, r.getExit(i)));
    
    double scoreP = max_distance - distance(pursuer,evader) + min_exit;
    double scoreE = max_distance + distance(pursuer,evader) - min_exit;
    return scoreP + scoreE;
}

Point2D PayoffMatrix::computeMove(Point2D pursuer, Point2D evader)
{
    pursuer = getNearestNode(pursuer, map.getNodes());
    evader = getNearestNode(evader, map.getNodes());
    double matrix_scores[KNN_MAX*KNN_MAX][KNN_MAX*KNN_MAX];
    Point2D path_p[KNN_MAX*KNN_MAX][2], path_e[KNN_MAX*KNN_MAX][2];
    int possible_p=0, possible_e=0;
    
    std::vector<Point2D> temp1, temp2;
    map.getAttachedNodes(pursuer, &temp1);
    for(int i=0; i<(int)temp1.size(); i++)
    {
        map.getAttachedNodes(temp1.at(i), &temp2);
        for(int j=0; j<(int)temp2.size(); j++)
        {
            path_p[i*temp1.size()+j][0]=temp1.at(i);
            path_p[i*temp1.size()+j][1]=temp2.at(j);
            possible_p++;
        }
    }

    map.getAttachedNodes(evader, &temp1);
    for(int i=0; i<(int)temp1.size(); i++)
    {
        map.getAttachedNodes(temp1.at(i), &temp2);
        for(int j=0; j<(int)temp2.size(); j++)
        {
            path_e[i*temp1.size()+j][0]=temp1.at(i);
            path_e[i*temp1.size()+j][1]=temp2.at(j);
            possible_e++;
        }
    }


    double max_score=0;
    int index_move_p, index_move_e;
    for(int i=0; i<possible_p; i++)
    {
        for(int j=0; j<possible_e; j++)
        {
            matrix_scores[i][j]=score(path_p[i][1],path_e[j][1], map.getRoom());
            if(matrix_scores[i][j]>max_score)
            {
                max_score=matrix_scores[i][j];
                index_move_p=i;
                index_move_e=j;
            }
        }
    }

    cerr << "Moves for the pursuer: " << path_p[index_move_p][0] << " " << path_p[index_move_p][1] << endl;
    cerr << "Moves for the evader: " << path_e[index_move_e][0] << " " << path_e[index_move_e][1] << endl;

    return path_p[index_move_p][0];
}