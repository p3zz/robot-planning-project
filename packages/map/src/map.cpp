#include "map/map.hpp"
using namespace std;

bool link_exists(Point2D p1, Point2D p2, std::vector<Segment2D> links)
{
    for(int i=0; i<(int)links.size(); i++)
        if(links.at(i).p1.x == p2.x && links.at(i).p1.y == p2.y && links.at(i).p2.x == p1.x && links.at(i).p2.y == p1.y)
            return true;
    return false;
}

bool link_collides(Segment2D link, Room r)
{
    for(int i=0; i<r.get_num_obstacles(); i++)
        if(intersect(r.get_obstacle(i), link))
            return true;
    return false;
}

bool point_collides(Point2D p, Room r)
{
    for(int i=0; i<r.get_num_obstacles(); i++)
        if(r.get_inflated_obstacle(i).contains(p))
            return true;
    return false;
}

bool check_sparse(Point2D p, std::vector<Point2D> nodes, double distance_min)
{
    for (int i = 0; i < (int)nodes.size(); i++) //check distance with other nodes
        if(distance(p, nodes[i])<distance_min) 
            return false;
    return true;    
}

void sort_knn(double arr1[], Point2D* arr2, int k)
{
    for(int j=0;j<k-1;j++)
		for(int i=0;i<k-1;i++)
            if(arr1[i]>arr1[i+1])
			{
                double temp1=arr1[i];
                arr1[i]=arr1[i+1];
                arr1[i+1]=temp1;
                Point2D temp2=arr2[i];
                arr2[i]=arr2[i+1];
                arr2[i+1]=temp2;
            }
}

void Knn(Point2D node, std::vector<Point2D> candidates, int k, Point2D* nearest_nodes, Room r)
{
    //Init variables
    double* nearest_distances = new double[k];
    for(int i=0; i<k; i++)
        nearest_distances[i]=POINT_COORD_MAX;
    
    //Test all candidates
    for(auto &cand: candidates){
        //different nodes
        if(cand.x != node.x && cand.y != node.y){
            //check if a node is better than others (starting from worst node)
            double d = distance(cand, node);
            for(int j=k-1; j>=0; j--)
                if(d<nearest_distances[j])
                {
                    if(!link_collides(Segment2D(node, cand),r))
                    { 
                        nearest_distances[j]=d;
                        nearest_nodes[j]=Point2D(cand.x, cand.y);
                        sort_knn(nearest_distances, nearest_nodes, k);
                    }
                    break;
                }
        }
    }
}

Room::Room(Polygon dimensions):dimensions{dimensions}
{
    //get approximate dimensions (rounded up to a rectangle)
    double min_x=999999, min_y=999999, max_x=-999999, max_y=-999999;
    for(auto &v: dimensions.vertexes)
    {
        if(v.x < min_x) min_x=v.x;
        if(v.x > max_x) max_x=v.x;
        if(v.y < min_y) min_y=v.y;
        if(v.y > max_y) max_y=v.y;
    }
    off_x = min_x;
    off_y = min_y;
    approx_h = max_y-min_y;
    approx_w = max_x-min_x;
    approx_area=0;

    //get approximate area (rounded down to small squares of 1 cm2)
    for(double p_x=min_x; p_x<=max_x; p_x+=0.01)
        for(double p_y=min_y; p_y<=max_y; p_y+=0.01)
            if(dimensions.contains(Point2D(p_x,p_y)))
                approx_area+=0.0001;
    
    //inflate room
    dimensions_deflated = deflate(dimensions, ROBOT_CIRCLE);
}

void Room::add_exit(Point2D exit)
{
    exits.push_back(exit);
    Segment2D s = belong(dimensions, exit, 0.1);
    double th = mod2pi(atan2(s.p2.y-s.p1.y, s.p2.x-s.p1.x)+1.5*M_PI);
    Point2D exit_inflated = translate(exit, ROBOT_CIRCLE*1.2, th);
    exits_inflated.push_back(exit_inflated);
}

//PRM ROADMAP
bool RoadMap::construct_roadmap(int points, int knn, double k_distance_init, double tms_max, Point2D p_pos, Point2D e_pos)
{
    if(knn>KNN_MAX) return false;

    //Add exit-pursuer-evader nodes
    for(int i=0; i<r.get_num_exits(); i++)
        nodes.push_back(r.get_exit(i, true));
    nodes.push_back(p_pos); //add pursuer position as a node
    nodes.push_back(e_pos); //add evader position as a node

    //Create nodes
    srand(Seed::get_seed());
    const double k_room_space=r.get_approx_area()/points;
    const double k_base=std::pow(0.1,1/tms_max); //base to decrease k_distance to 10% at tms_max
    
    for (int i=2; i<points; i++)
    {
        double tms,k_distance,distance_pts;
        Point2D node;
        int count=0;
        const clock_t begin_time = clock();
        do{
            if(count==0)
            {
                count=100; //number of retries with same distance_pts
                tms=(float(clock()-begin_time)/CLOCKS_PER_SEC)*1000;
                k_distance=std::pow(k_base,tms)*k_distance_init;//bigger is k_distance, more homogeneus the map, much diffcult the spawning of points
                distance_pts=k_room_space*k_distance;
            }
            node.x = int(rand() % int(r.get_approx_width()*100)) / 100.0 + r.get_offset_x();  //cm sensibility, consider room border
            node.y = int(rand() % int(r.get_approx_height()*100)) / 100.0 + r.get_offset_y(); //cm sensibility, consider room border
            count--;
        }while(!check_sparse(node, nodes, distance_pts) || point_collides(node, this->r) || !r.get_dimensions(true).contains(node)); //check for sparse nodes
        nodes.push_back(node);
    }
    
    //Create links
    for (auto &node: nodes)
    {
        Point2D node_knn[KNN_MAX];
        for(int j=0; j<knn; j++)
            node_knn[j]=Point2D(POINT_COORD_MAX, POINT_COORD_MAX);
        Knn(node, nodes, knn, node_knn, this->r);

        //Insert link in the vector
        for (int j = 0; j < knn; j++)
            if(node_knn[j].x!=POINT_COORD_MAX && node_knn[j].y!=POINT_COORD_MAX && !link_exists(node, node_knn[j], links))
                links.push_back(Segment2D(Point2D(node.x,node.y),Point2D(node_knn[j].x, node_knn[j].y)));      
    }

    //Create Dubins Curves
    auto obstacles = r.get_inflated_obstacles();
    auto dimensions_room = r.get_dimensions(true);
    for(auto &link: links){
        std::vector<DubinLink> d_links;
        // compute every dubins curve for each M_PI/4 
        double step = M_PI * 0.25;
        for(double th_src = 0; th_src < 2 * M_PI; th_src += step){
            DubinPoint node1(link.p1.x, link.p1.y, th_src);
            for(double th_dst = 0; th_dst < 2 * M_PI; th_dst += step){
                DubinPoint node2(link.p2.x, link.p2.y, th_dst);
                // compute dubins from node1 to node2
                auto curves = dubin_curves(node1, node2, MAX_CURVATURE);
                for(auto &curve: curves){
                    bool inter = intersect_sides(curve, dimensions_room);
                    if(!inter)
                        for(auto &ob: obstacles){
                            if(intersect(curve, ob)){
                                inter = true;
                                break;
                            }
                        }
                    // if the curve doesn't collide with any obstacle, keep this as best curve for the pair (src, dst)
                    if(!inter){
                        dubin_links.push_back(DubinLink(node1, node2, curve));
                        break;
                    }
                }

                // compute dubins from node2 to node1
                curves = dubin_curves(node2, node1, MAX_CURVATURE);
                for(auto &curve: curves){
                    bool inter = intersect_sides(curve, dimensions_room);
                    if(!inter)
                        for(auto &ob: obstacles){
                            if(intersect(curve, ob)){
                                inter = true;
                                break;
                            }
                        }
                    if(!inter){
                        dubin_links.push_back(DubinLink(node2, node1, curve));
                        break;
                    }
                }
            }
        }
    }

    return true;
}

std::string RoadMap::get_json()
{
    std::string json="";

    json += "{\"room\": {\"h\":"+std::to_string(r.get_approx_height())+",\"w\":"+std::to_string(r.get_approx_width())+",\"min_x\":"+std::to_string(r.get_offset_x())+",\"min_y\":"+std::to_string(r.get_offset_y());
    json += ",\"vertexes\": [";
    for(int i=0; i<(int)r.get_dimensions(false).vertexes.size();i++)
    {
        json+="{\"x\":"+std::to_string(r.get_dimensions(false).get_v(i).x)+",\"y\":"+std::to_string(r.get_dimensions(false).get_v(i).y)+"}";
        if(i+1<(int)r.get_dimensions(false).vertexes.size()) json+=",";
    }
    json += "],\"vertexes_deflated\": [";
    for(int i=0; i<(int)r.get_dimensions(true).vertexes.size();i++)
    {
        json+="{\"x\":"+std::to_string(r.get_dimensions(true).get_v(i).x)+",\"y\":"+std::to_string(r.get_dimensions(true).get_v(i).y)+"}";
        if(i+1<(int)r.get_dimensions(true).vertexes.size()) json+=",";
    }
    
    json += "]},\"roadmap\":";
    json += "{\"nodes\":[";
    for(int i=0; i<(int)nodes.size();i++)
    {
        json+="{\"x\":"+std::to_string(nodes[i].x)+",\"y\":"+std::to_string(nodes[i].y)+"}";
        if(i+1<(int)nodes.size()) json+=",";
    }
    json+="],\"links\":[";
    for(int i=0; i<(int)links.size();i++)
    {
        json+="{\"node1\":{\"x\":"+std::to_string(links[i].p1.x)+",\"y\":"+std::to_string(links[i].p1.y)+"},\"node2\":{\"x\":"+std::to_string(links[i].p2.x)+",\"y\":"+std::to_string(links[i].p2.y)+"}}";
        if(i+1<(int)links.size()) json+=",";
    }
    json+="],\"obstacles\":[";
    for(int i=0; i<r.get_num_obstacles();i++)
    {
        json+="{\"normal\":[";
        for(int j=0;j<r.get_obstacle(i).get_size();j++)
        {
            json+="{\"x\":"+std::to_string(r.get_obstacle(i).get_v(j).x)+",\"y\":"+std::to_string(r.get_obstacle(i).get_v(j).y)+"}";
            if(j+1<r.get_obstacle(i).get_size()) json+=",";
        }
        json+="],\"inflated\":[";
        for(int j=0;j<r.get_inflated_obstacle(i).get_size();j++)
        {
            json+="{\"x\":"+std::to_string(r.get_inflated_obstacle(i).get_v(j).x)+",\"y\":"+std::to_string(r.get_inflated_obstacle(i).get_v(j).y)+"}";
            if(j+1<r.get_inflated_obstacle(i).get_size()) json+=",";
        }
        json+="]}";
        if(i+1<r.get_num_obstacles()) json+=",";
    }
    json+="],\"exits\":[";
    for(int i=0; i<r.get_num_exits();i++)
    {
        json+="{\"normal\": {\"x\":"+std::to_string(r.get_exit(i, false).x)+",\"y\":"+std::to_string(r.get_exit(i, false).y)+"},";
        json+="\"inflated\": {\"x\":"+std::to_string(r.get_exit(i, true).x)+",\"y\":"+std::to_string(r.get_exit(i, true).y)+"}}";
        if(i+1<r.get_num_exits()) json+=",";
    }
    json+="]}}";
    return json;
}

void RoadMap::get_attached_nodes(Point2D node, std::vector<Point2D> *attached_links)
{
    attached_links->clear();
    for(auto &link: links){
        if(node==link.p1) attached_links->push_back(link.p2);
        else if(node==link.p2) attached_links->push_back(link.p1);
    }
}

DubinLink RoadMap::get_dubin_link(DubinPoint dp1, DubinPoint dp2)
{
    for(auto &link: dubin_links)
        if(link.get_src() == dp1 && link.get_dst() == dp2)
            return link;
    return DubinLink();
}

void random_obstacles_side(Room* room, int num_obstacles, const int max_side)
{
    Point2D* centers = new Point2D[num_obstacles];
    for(int i=0; i<num_obstacles; i++)
    {
        Polygon o;
        bool check;
        do{
        centers[i].x = ((rand()%(int)((room->get_approx_width())*100-max_side))+max_side/2)/100.0+room->get_offset_x();
        centers[i].y = ((rand()%(int)((room->get_approx_height())*100-max_side))+max_side/2)/100.0+room->get_offset_y();
        check=true;
        if(!room->get_dimensions(true).contains(centers[i]))
            check=false;
        for(int j=0;j<i && check;j++)
            if(distance(centers[j], centers[i])<max_side/100.0*sqrt(2))
            {
            check=false;
            break;
            }
        }while(!check);
        Point2D center(centers[i]);
        o.add_v(Point2D(center.x+((rand() %(max_side/4))-(max_side/2))/100.0,center.y+((rand() %(max_side/4))+(max_side/4))/100.0));
        o.add_v(Point2D(center.x+((rand() %(max_side/4))+(max_side/4))/100.0,center.y+((rand() %(max_side/4))+(max_side/4))/100.0));
        o.add_v(Point2D(center.x+((rand() %(max_side/4))+(max_side/4))/100.0,center.y+((rand() %(max_side/4))-(max_side/2))/100.0));
        o.add_v(Point2D(center.x+((rand() %(max_side/4))-(max_side/2))/100.0,center.y+((rand() %(max_side/4))-(max_side/2))/100.0));
        room->add_obstacle(o);
    }
}

void random_obstacles_vertexes(Room* room, int num_obstacles, int vertexes_n){
    random_device rd;

    mt19937 gen(rd());

    uniform_real_distribution<double> x_dis(0, room->get_approx_width());
    uniform_real_distribution<double> y_dis(0, room->get_approx_height());
    
    // distance between the center of the polygon and one of its vertexes
    double const radius = 0.5;
    Point2D* centers = new Point2D[num_obstacles];
    for(int i=0; i<num_obstacles; i++){
        bool valid = true;
        do {
            centers[i].x = x_dis(gen);
            centers[i].y = y_dis(gen);
            valid = true;
            
            if(i>0){
                for(int j=0;j<i;j++){
                // check if the centers are correctly distanciated
                    if(distance(centers[j], centers[i]) < radius * 2){
                        valid = false;
                        break;
                    }
                }
            }
        } while(!valid);
        Polygon obstacle = regular_polygon(centers[i], radius, vertexes_n);
        room->add_obstacle(obstacle);
    }
}

int RoadMap::get_node_index(Point2D p){
    auto nodes = get_nodes();
    for(int i = 0; i < (int)nodes.size(); i++){
        if(nodes[i] == p){
            return i;
        }
    }
    return -1;
}