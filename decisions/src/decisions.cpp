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

void PayoffMatrix::computeMove(Point2D pursuer, Point2D evader, Point2D& move_pursuer, Point2D& move_evader)
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

    //cerr << "Moves for the pursuer: " << pursuer << " " << path_p[index_move_p][0] << " " << path_p[index_move_p][1] << " with score " << max_score_p << endl;
    //cerr << "Moves for the evader: " << evader << " " << path_e[index_move_e][0] << " " << path_e[index_move_e][1] << " with score " << max_score_e << endl;

    move_pursuer=path_p[index_move_p][0];
    move_evader=path_e[index_move_e][0];
}

double moves_robots(Point2D pursuer_0, Point2D pursuer_1, Point2D evader_0, Point2D evader_1, Point2D& pursuer_final, Point2D& evader_final)
{
    double tp=distance(pursuer_0, pursuer_1)/ROBOT_VELOCITY;
    double te=distance(evader_0, evader_1)/ROBOT_VELOCITY;
    if(tp>te)
    {
        evader_final = evader_1;
        pursuer_final.x = (pursuer_1.x-pursuer_0.x)*(te/tp) + pursuer_0.x; 
        pursuer_final.y = (pursuer_1.y-pursuer_0.y)*(te/tp) + pursuer_0.y; 
        return te;
    }
    else
    {
        pursuer_final = pursuer_1;
        evader_final.x = (evader_1.x-evader_0.x)*(tp/te) + evader_0.x; 
        evader_final.y = (evader_1.y-evader_0.y)*(tp/te) + evader_0.y; 
        return tp;
    }
}

void simulate(Point2D pursuer_pos, Point2D evader_pos, PayoffMatrix decision)
{
    ofstream myfile;
    myfile.open ("moves.json", std::ofstream::trunc);
    myfile<<"{\"moves\":[{\"pursuer\":{\"x\":"<<pursuer_pos.x<<",\"y\":"<<pursuer_pos.y<<"},\"evader\":{\"x\":"<<evader_pos.x<<",\"y\":"<<evader_pos.y<<"}}";
    Point2D pursuer_1, evader_1, pursuer_half, evader_half, throw_pos;
    bool end=false;
    int next_move=0, count=0;
    double t=0;
    while(!end && count < 20)
    {
        switch(next_move)
        {
            case 1:
                decision.computeMove(pursuer_pos, evader_1, pursuer_1, throw_pos);
                break;
            case 2:
                decision.computeMove(pursuer_1, evader_pos, throw_pos, evader_1);
                break;
            default:
                decision.computeMove(pursuer_pos, evader_pos, pursuer_1, evader_1);
        }
        
        t+=moves_robots(pursuer_pos, pursuer_1, evader_pos, evader_1, pursuer_half, evader_half);
        myfile << ",{\"pursuer\":{\"x\":"<<pursuer_half.x<<",\"y\":"<<pursuer_half.y<<"},\"evader\":{\"x\":"<<evader_half.x<<",\"y\":"<<evader_half.y<<"}}";
        
        if(pursuer_half == pursuer_1) //pursuer completed the move
        {
            if(pursuer_half == evader_1)
            {
                end=true;
                cout << "Pursuer reach evader position! YOU WIN" << endl;
            }
            next_move=1;
            pursuer_pos=pursuer_1;
            evader_pos=evader_half;
        }
        else //evader completed the move
        {
            for(int i=0; i<decision.map.getRoom().getNumExits(); i++)
                if(evader_half==decision.map.getRoom().getExit(i))
                {
                    end=true;
                    cout << "Evader evaded from the room! YOU LOSE" << endl;
                }
            next_move=2;
            evader_pos=evader_1;
            pursuer_pos=pursuer_half;
        }
        count++;
    }
    myfile << "]}";
    cout << "total time: " << t << endl;
}