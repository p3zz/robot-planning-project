#include "planner/planner.hpp"
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

DubinPoint get_nearest_node(DubinPoint position, std::vector<Point2D> nodes, bool angle_approx)
{
    DubinPoint p;
    double dist_min=POINT_COORD_MAX, dist;
    for(int i=0; i<(int)nodes.size(); i++)
    {
        dist = distance(position.get_point(), nodes.at(i));

        if(dist<dist_min)
        {
            bool filtered=false;
            //filter for other case: approximation with angle
            if(angle_approx && dist>=0.1) //if set, choose only points in direction of robot +-60° or very close points (10 cm)
            {
                double diff_angle=modpi(position.th - atan2(nodes.at(i).y-position.y, nodes.at(i).x-position.x));
                filtered = diff_angle < -M_PI/3 || diff_angle > M_PI/3;

            }
            if(!filtered)
            {
                dist_min = dist;
                p=DubinPoint(nodes.at(i),angle_in_range(position.th));
            }
        }
    }
    return p;
}

double score_angle(DubinPoint dp1, Point2D p2) //score from 0 to 1, based on proximity of angles
{
    if(distance(dp1.get_point(), p2) < 0.1)
        return 1;
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
    for(int i=0; i<r.get_room().get_num_exits(); i++)
    {
        double d=distance(path_e.p2.get_point(), r.get_room().get_exit(i, false));
        if(d<min_exit)
        {
            min_exit=d;
            score_th=score_angle(path_e.p2, r.get_room().get_exit(i, false));
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
    for(int i=0; i<r.get_room().get_num_exits(); i++) //more probable exit chosen by evader
    {
        double d=distance(evader_2, r.get_room().get_exit(i,false));
        if(d<min_exit)
        {
            min_exit=d;
            index_exit=i;
        }
    }
    //Distance between pursuer and exit
    double distanceExit = distance(path_p.p2.get_point(), r.get_room().get_exit(index_exit, false));
    //Score for direction
    double score_th = score_angle(path_p.p2, avg_point(evader_2, r.get_room().get_exit(index_exit, false)));
    
    double scoreP = center - distancePE *1.5 - distanceExit *0.75 - len_path *0.75 + score_th * 4;
    return scoreP;
}

bool PayoffMatrix::compute_move(DubinPoint pursuer, DubinPoint evader, Path& path_pursuer, Path& path_evader)
{
    pursuer = get_nearest_node(pursuer, map.get_nodes(), false);
    evader = get_nearest_node(evader, map.get_nodes(), true);
    std::vector<Path> path_p, path_e;
    std::vector<Point2D> possible_p;

    //Compute possible moves for pursuer (2 forward)
    std::vector<Point2D> temp1, temp2;
    map.get_attached_nodes(pursuer.get_point(), &temp1);
    for(int i=0; i<(int)temp1.size(); i++)
    {
        map.get_attached_nodes(temp1.at(i), &temp2);
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
    map.get_attached_nodes(evader.get_point(), &temp1);
    for(int i=0; i<(int)temp1.size(); i++)
    {
        map.get_attached_nodes(temp1.at(i), &temp2);
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
        Logger(Logger::ERROR, "No moves found for evader");
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
    if(index_move_p==-1)
    {
        Logger(Logger::ERROR, "No moves found for pursuer");
        return false;
    }
    
    path_pursuer=path_p.at(index_move_p);
    path_evader=path_e.at(index_move_e);

    return true;
}

std::string get_pursuer_evader_path_json(Path path_pursuer, Path path_evader, double precision){
    std::string s;

    s += "{";

    std::vector<DubinPoint> points = path_pursuer.l1.get_curve().to_points_homogeneous(precision);
    s += "\"moves_pursuer\":";
    s += "[";
    s += path_pursuer.to_json(precision);
    s += "],";
    
    s += "\"moves_evader\":";
    s += "[";
    s += path_evader.to_json(precision);
    s += "]";
    s += "}";

    return s;   
}

std::string get_pursuer_evader_moves_json(std::vector<DubinLink> pursuer_moves, std::vector<DubinLink> evader_moves, double precision){
    std::string s;

    s += "{";

    s += "\"moves_pursuer\":";
    s += "[";
    auto index = 0;
    for(auto &move: pursuer_moves){
        s += move.to_json(precision);
        if(index != (int)pursuer_moves.size() - 1){
            s += ",";
        }
        index++;
    }
    s += "],";
    
    s += "\"moves_evader\":";
    s += "[";
    index = 0;
    for(auto &move: evader_moves){
        s += move.to_json(precision);
        if(index != (int)evader_moves.size() - 1){
            s += ",";
        }
        index++;
    }
    s += "]";
    s += "}";

    return s;   
}

string operator + (string s, Path& path)
{
    DubinCurve first=path.l1.get_curve();
    DubinCurve second=path.l2.get_curve();
    return s+" -"+first+"-> "+path.p1+" -"+second+"-> "+path.p2;
}

std::string Path::to_json(double precision){
    return l1.to_json(precision) + "," + l2.to_json(precision);
}