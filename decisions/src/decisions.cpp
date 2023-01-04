#include "decisions/decisions.hpp"
using namespace std;

double min(double n1, double n2)
{
    return n2<n1? n2:n1;
}

double angle_in_range(double angle)
{
    angle=mod2pi(angle);
    double nearest_value=100, nearest_angle;
    for(int i=0; i<8; i++)
    {
        double tang=abs(modpi(i*0.25*M_PI-angle));
        if(tang<nearest_value)
        {
            nearest_angle=i*0.25*M_PI;
            nearest_value=tang;
        }
    }
    return nearest_angle;
}

DubinPoint getNearestNode(DubinPoint position, std::vector<Point2D> nodes)
{
    DubinPoint p;
    double dist_min=POINT_COORD_MAX, dist;
    for(int i=0; i<(int)nodes.size(); i++)
    {
        dist=distance(position.get_point(), nodes.at(i));
        if(dist<dist_min)
        {
            dist_min=dist;
            p=DubinPoint(nodes.at(i),angle_in_range(position.th));
        }
    }
    return p;
}

double score_angle(DubinPoint dp1, Point2D p2) //score from 0 to 1, based on proximity of angles
{
    double angle = atan2(p2.y-dp1.y, p2.x-dp1.x);
    return 1-abs(modpi(dp1.th-angle))/M_PI;
}

double scoreE(Point2D pursuer_2, Path path_e, RoadMap& r)
{
    //Return 0 score if there is no dubins (collision)
    if(path_e.l1.is_empty() || path_e.l2.is_empty())
        return 0;
    //Score must be positive
    double center=1000;
    //Length of trajectory
    double len_path = path_e.l1.get_curve().get_length() + path_e.l2.get_curve().get_length();
    //Distance between pursuer and evader
    double distancePE = distance(pursuer_2, path_e.p2.get_point());
    //Min Distance from exit & Score for direction
    double min_exit=POINT_COORD_MAX;
    double score_th=0;
    for(int i=0; i<r.getRoom().getNumExits(); i++)
    {
        double d=distance(path_e.p2.get_point(), r.getRoom().getExit(i));
        if(d<min_exit)
        {
            min_exit=d;
            score_th=score_angle(path_e.p2, r.getRoom().getExit(i));
        }
    }

    double scoreE = center + distancePE - min_exit - len_path + score_th * 4;
    return scoreE;
}

double scoreP(Point2D evader_2, Path path_p, RoadMap& r)
{
    //Return 0 score if there is no dubins (collision)
    if(path_p.l1.is_empty() || path_p.l2.is_empty())
        return 0;
    //Score must be positive
    double center=1000;
    //Length of trajectory
    double len_path = path_p.l1.get_curve().get_length() + path_p.l2.get_curve().get_length();
    //Distance between pursuer and evader
    double distancePE = distance(evader_2, path_p.p2.get_point());
    //Probable exit chosen by evader
    double min_exit=POINT_COORD_MAX;
    int index_exit;
    for(int i=0; i<r.getRoom().getNumExits(); i++) //more probable exit chosen by evader
    {
        double d=distance(evader_2, r.getRoom().getExit(i));
        if(d<min_exit)
        {
            min_exit=d;
            index_exit=i;
        }
    }
    //Distance between pursuer and exit
    double distanceExit = distance(path_p.p2.get_point(), r.getRoom().getExit(index_exit));
    //Score for direction
    double score_th = score_angle(path_p.p2, avg_point(evader_2, r.getRoom().getExit(index_exit)));
    
    double scoreP = center - distancePE *1.5 - distanceExit *0.75 - len_path *0.75 + score_th * 4;
    return scoreP;
}

bool PayoffMatrix::computeMove(DubinPoint pursuer, DubinPoint evader, Path& path_pursuer, Path& path_evader)
{
    pursuer = getNearestNode(pursuer, map.getNodes());
    evader = getNearestNode(evader, map.getNodes());
    std::vector<Path> path_p, path_e;
    std::vector<Point2D> possible_p;

    clock_t now = clock();

    //Compute possible moves for pursuer (2 forward)
    std::vector<Point2D> temp1, temp2;
    map.getAttachedNodes(pursuer.get_point(), &temp1);
    for(int i=0; i<(int)temp1.size(); i++)
    {
        map.getAttachedNodes(temp1.at(i), &temp2);
        for(int j=0; j<(int)temp2.size(); j++)
        {
            possible_p.push_back(temp2.at(j));
            double angle=angle_in_range(atan2(temp2.at(j).y-temp1.at(i).y, temp2.at(j).x-temp1.at(i).x));
            DubinPoint p1 = DubinPoint(temp1.at(i), angle);
            DubinLink l1 = map.get_dubin_link(pursuer, p1);
            for(int k=0; k<8; k++)
            {
                Path t;
                t.p1 = p1;
                t.l1 = l1;
                t.p2 = DubinPoint(temp2.at(j), M_PI*0.25*k);
                t.l2 = map.get_dubin_link(t.p1, t.p2);
                path_p.push_back(t);
            }
        }
    }    

    //Compute possible moves for evader (2 forward)
    map.getAttachedNodes(evader.get_point(), &temp1);
    for(int i=0; i<(int)temp1.size(); i++)
    {
        map.getAttachedNodes(temp1.at(i), &temp2);
        for(int j=0; j<(int)temp2.size(); j++)
        {
            double angle=angle_in_range(atan2(temp2.at(j).y-temp1.at(i).y, temp2.at(j).x-temp1.at(i).x));
            DubinPoint p1 = DubinPoint(temp1.at(i), angle);
            DubinLink l1 = map.get_dubin_link(evader, p1);
            for(int k=0; k<8; k++)
            {
                Path t;
                t.p1 = p1;
                t.l1 = l1;
                t.p2 = DubinPoint(temp2.at(j), M_PI*0.25*k);
                t.l2 = map.get_dubin_link(t.p1, t.p2);
                path_e.push_back(t);
            }
        }
    }

    //Matrix with all possible combinations and selection of high score of evader (let evader decide with best intelligence, then compute our move)
    double score, max_score_e=0, max_score_p=0;
    int index_move_p=-1, index_move_e=-1;
    for(int i=0; i<(int)possible_p.size(); i++)
    {
        for(int j=0; j<(int)path_e.size(); j++)
        {
            //Compute score for evader
            score = scoreE(possible_p.at(i), path_e.at(j), map);
            if(score > max_score_e)
            {
                max_score_e = score;
                index_move_e = j;
            }
        }
    }
    if(index_move_e==-1)
    {
        cerr << "No moves found for evader";
        return false;
    }


    //Best move for the pursuer
    for(int i=0; i<(int)path_p.size(); i++)
    {
        score = scoreP(path_e.at(index_move_e).p2.get_point(), path_p.at(i), map);
        if(score > max_score_p)
        {
            max_score_p = score;
            index_move_p = i;
        }
    }
    if(index_move_e==-1)
    {
        cerr << "No moves found for pursuer";
        return false;
    }
    
    path_pursuer=path_p.at(index_move_p);
    path_evader=path_e.at(index_move_e);


    cout << "Completed in " << ((clock()-now)/(double)CLOCKS_PER_SEC)*1000 << " ms" << endl;

    ofstream myfile;
    myfile.open ("moves.json", std::ofstream::trunc);
    myfile << "{\"moves\":[{\"pursuer\":{\"x\":"<<pursuer.x<<",\"y\":"<<pursuer.y<<"},\"evader\":{\"x\":"<<evader.x<<",\"y\":"<<evader.y<<"}},";
    myfile <<             "{\"pursuer\":{\"x\":"<<path_p.at(index_move_p).p1.x<<",\"y\":"<<path_p.at(index_move_p).p1.y<<"},\"evader\":{\"x\":"<<path_e.at(index_move_e).p1.x<<",\"y\":"<<path_e.at(index_move_e).p1.y<<"}},";
    myfile <<             "{\"pursuer\":{\"x\":"<<path_p.at(index_move_p).p2.x<<",\"y\":"<<path_p.at(index_move_p).p2.y<<"},\"evader\":{\"x\":"<<path_e.at(index_move_e).p2.x<<",\"y\":"<<path_e.at(index_move_e).p2.y<<"}}]}";
    myfile.close();

    return true;
}




double moves_robots(Point2D pursuer_0, Point2D pursuer_1, Point2D evader_0, Point2D evader_1, Point2D& pursuer_final, Point2D& evader_final)
{
    double t_s1, t_s2;
    if(intersect(Segment(pursuer_0, pursuer_1), Segment(evader_0, evader_1), t_s1, t_s2))
    {
        pursuer_final.x = (pursuer_1.x-pursuer_0.x)*t_s1+pursuer_0.x;
        pursuer_final.y = (pursuer_1.y-pursuer_0.y)*t_s1+pursuer_0.y;
        evader_final = pursuer_final;
        double tp=distance(pursuer_0, pursuer_final)/ROBOT_VELOCITY;
        double te=distance(evader_0, evader_final)/ROBOT_VELOCITY;
        return tp>te?tp:te;
    }
    else
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
    while(!end && count < 200)
    {
        switch(next_move)
        {
            case 1:
                //decision.computeMove(pursuer_pos, evader_1, pursuer_1, throw_pos);
                break;
            case 2:
                //decision.computeMove(pursuer_1, evader_pos, throw_pos, evader_1);
                break;
            default:
                //decision.computeMove(pursuer_pos, evader_pos, pursuer_1, evader_1);
                break;
        }
        
        t+=moves_robots(pursuer_pos, pursuer_1, evader_pos, evader_1, pursuer_half, evader_half);
        myfile << ",{\"pursuer\":{\"x\":"<<pursuer_half.x<<",\"y\":"<<pursuer_half.y<<"},\"evader\":{\"x\":"<<evader_half.x<<",\"y\":"<<evader_half.y<<"}}";
        
        if(pursuer_half == evader_half) //case intersection
        {
            end=true;
            cout << "Pursuer reach evader position! YOU WIN" << endl;
        }
        else if(pursuer_half == pursuer_1) //pursuer completed the move
        {
            /*if(pursuer_half == evader_1)
            {
                end=true;
                cout << "Pursuer reach evader position! YOU WIN" << endl;
            }*/
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