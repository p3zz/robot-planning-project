#include "decisions/decisions.hpp"
using namespace std;

double min(double n1, double n2)
{
    return n2<n1? n2:n1;
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

double scoreE(Point2D pursuer_2, Point2D evader_0, Point2D evader_1, Point2D evader_2, Room& r)
{
    double max_distance=sqrt(pow(r.getHeight(),2)+pow(r.getWidth(),2));
    double len_path = distance(evader_1, evader_0) + distance(evader_2, evader_1);
    double min_exit=POINT_COORD_MAX;
    for(int i=0; i<r.getNumExits(); i++)
        min_exit=min(min_exit, distance(evader_2, r.getExit(i)));

    double scoreE = max_distance*2 + distance(pursuer_2, evader_2) - min_exit - len_path;
    return scoreE;
}

double scoreP(Point2D evader_2, Point2D pursuer_0, Point2D pursuer_1, Point2D pursuer_2, Room& r)
{
    double max_distance=sqrt(pow(r.getHeight(),2)+pow(r.getWidth(),2));
    double len_path = distance(pursuer_1, pursuer_0) + distance(pursuer_2, pursuer_1);
    double min_exit=POINT_COORD_MAX;
    int index_exit;
    for(int i=0; i<r.getNumExits(); i++) //more probable exit chosen by evader
        if(distance(evader_2, r.getExit(i))<min_exit)
        {
            min_exit=distance(evader_2, r.getExit(i));
            index_exit=i;
        }

    
    double scoreP = max_distance*3 - distance(pursuer_2, evader_2) *1.5 - distance(pursuer_2, r.getExit(index_exit)) *0.75 - len_path *0.75;
    return scoreP;
}

Point2D PayoffMatrix::computeMove(Point2D pursuer, Point2D evader)
{
    pursuer = getNearestNode(pursuer, map.getNodes());
    evader = getNearestNode(evader, map.getNodes());
    Point2D path_p[KNN_MAX*KNN_MAX][2], path_e[KNN_MAX*KNN_MAX][2];
    int possible_p=0, possible_e=0;
    
    //Compute possible moves for pursuer (2 forward)
    std::vector<Point2D> temp1, temp2;
    map.getAttachedNodes(pursuer, &temp1);
    for(int i=0; i<(int)temp1.size(); i++)
    {
        map.getAttachedNodes(temp1.at(i), &temp2);
        for(int j=0; j<(int)temp2.size(); j++)
        {
            path_p[possible_p][0]=temp1.at(i);
            path_p[possible_p][1]=temp2.at(j);
            possible_p++;
        }
    }

    //Compute possible moves for evader (2 forward)
    map.getAttachedNodes(evader, &temp1);
    for(int i=0; i<(int)temp1.size(); i++)
    {
        map.getAttachedNodes(temp1.at(i), &temp2);
        for(int j=0; j<(int)temp2.size(); j++)
        {
            path_e[possible_e][0]=temp1.at(i);
            path_e[possible_e][1]=temp2.at(j);
            possible_e++;
        }
    }

    //Matrix with all possible combinations and selection of high score of evader (let evader decide with best intelligence, then compute our move)
    double score, max_score_e=0, max_score_p=0;
    int index_move_p, index_move_e;
    for(int i=0; i<possible_p; i++)
    {
        for(int j=0; j<possible_e; j++)
        {
            score = scoreE(path_p[i][1], evader, path_e[j][0], path_e[j][1], map.getRoom());
            if(score > max_score_e)
            {
                max_score_e = score;
                index_move_e = j;
            }
        }
    }
    //Best move for the pursuer
    for(int i=0; i<possible_p; i++)
    {
        score = scoreP(path_e[index_move_e][1], pursuer, path_p[i][0], path_p[i][1], map.getRoom());
        if(score > max_score_p)
        {
            max_score_p = score;
            index_move_p = i;
        }
    }

    cerr << "Moves for the pursuer: " << pursuer << " " << path_p[index_move_p][0] << " " << path_p[index_move_p][1] << " with score " << max_score_p << endl;
    cerr << "Moves for the evader: " << evader << " " << path_e[index_move_e][0] << " " << path_e[index_move_e][1] << " with score " << max_score_e << endl;

    return path_p[index_move_p][0]; //return first move
}