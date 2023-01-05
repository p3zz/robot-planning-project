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

    return true;
}

std::string get_path_json(Path& path_pursuer, Path& path_evader, double precision)
{
    std::string s;

    s += "{";

    std::vector<DubinPoint> points = path_pursuer.l1.get_curve().to_points_homogeneus(precision);
    s +=    "\"moves_pursuer\":";
    s +=        "[";
    for(int i=0; i<(int)points.size(); i++)
    {
    s +=            "{";
    s +=                "\"x\":"+to_string(points.at(i).x)+",";
    s +=                "\"y\":"+to_string(points.at(i).y)+",";
    s +=                "\"marked\": false";
    s +=            "},";
    }
    points.clear();
    points = path_pursuer.l2.get_curve().to_points_homogeneus(precision);
    s +=            "{";
    s +=                "\"x\":"+to_string(path_pursuer.p1.x)+",";
    s +=                "\"y\":"+to_string(path_pursuer.p1.y)+",";
    s +=                "\"marked\": true";
    s +=            "},";
    for(int i=0; i<(int)points.size(); i++)
    {
    s +=            "{";
    s +=                "\"x\":"+to_string(points.at(i).x)+",";
    s +=                "\"y\":"+to_string(points.at(i).y)+",";
    s +=                "\"marked\": false";
    s +=            "},";
    }
    s +=            "{";
    s +=                "\"x\":"+to_string(path_pursuer.p2.x)+",";
    s +=                "\"y\":"+to_string(path_pursuer.p2.y)+",";
    s +=                "\"marked\": true";
    s +=            "}";
    s +=        "],";
    
    points.clear();
    points = path_evader.l1.get_curve().to_points_homogeneus(precision);
    s +=    "\"moves_evader\":";
    s +=        "[";
    for(int i=0; i<(int)points.size(); i++)
    {
    s +=            "{";
    s +=                "\"x\":"+to_string(points.at(i).x)+",";
    s +=                "\"y\":"+to_string(points.at(i).y)+",";
    s +=                "\"marked\": false";
    s +=            "},";
    }
    points.clear();
    points = path_evader.l2.get_curve().to_points_homogeneus(precision);
    s +=            "{";
    s +=                "\"x\":"+to_string(path_evader.p1.x)+",";
    s +=                "\"y\":"+to_string(path_evader.p1.y)+",";
    s +=                "\"marked\": true";
    s +=            "},";
    for(int i=0; i<(int)points.size(); i++)
    {
    s +=            "{";
    s +=                "\"x\":"+to_string(points.at(i).x)+",";
    s +=                "\"y\":"+to_string(points.at(i).y)+",";
    s +=                "\"marked\": false";
    s +=            "},";
    }
    s +=            "{";
    s +=                "\"x\":"+to_string(path_evader.p2.x)+",";
    s +=                "\"y\":"+to_string(path_evader.p2.y)+",";
    s +=                "\"marked\": true";
    s +=            "}";
    s +=        "]";

    s += "}";

    return s;   
}