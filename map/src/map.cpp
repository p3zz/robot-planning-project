#include "map/map.hpp"
using namespace std;

bool link_exists(Point2D node1, Point2D node2, std::vector<Segment> links)
{
    for(int i=0; i<(int)links.size(); i++)
        if(links.at(i).node1.x == node2.x && links.at(i).node1.y == node2.y && links.at(i).node2.x == node1.x && links.at(i).node2.y == node1.y)
            return true;
    return false;
}

bool link_collides(Segment link, Room r)
{
    for(int i=0; i<r.getNumObstacles(); i++)
        if(intersect(r.getObstacle(i), link))
            return true;
    return false;
}

bool point_collides(Point2D p, Room r)
{
    for(int i=0; i<r.getNumObstacles(); i++)
        if(r.getInflatedObstacle(i).contains(p))
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
    for(int i=0; i<(int)candidates.size(); i++)
    {
        Point2D cand = candidates.at(i);
        //different nodes
        if(cand.x != node.x && cand.y != node.y)
        {
            //check if a node is better than others (starting from worst node)
            double d=distance(cand, node);
            for(int j=k-1; j>=0; j--)
                if(d<nearest_distances[j])
                {
                    if(!link_collides(Segment(node, cand),r))
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

//PRM ROADMAP
bool RoadMap::constructRoadMap(int points, int knn, double k_distance_init, double tms_max)
{
    if(knn>KNN_MAX) return false;
    
    //Create nodes
    srand(time(NULL));
    const double k_room_space=(r.getHeight()-ROBOT_CIRCLE*2)*(r.getWidth()-ROBOT_CIRCLE*2)/points;
    const double k_base=std::pow(0.1,1/tms_max); //base to decrease k_distance to 10% at tms_max
    for (int i=0; i<points; i++)
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
            node.x = int(rand() % int((r.getWidth()-ROBOT_CIRCLE*2)*100)) / 100.0 + ROBOT_CIRCLE;  //cm sensibility, consider room border
            node.y = int(rand() % int((r.getHeight()-ROBOT_CIRCLE*2)*100)) / 100.0 + ROBOT_CIRCLE; //cm sensibility, consider room border
            count--;
        }while(!check_sparse(node, nodes, distance_pts) || point_collides(node, this->r)); //check for sparse nodes
        nodes.push_back(node);
    }

    //Add exit nodes
    for(int i=0; i<r.getNumExits(); i++)
        nodes.push_back(r.getExit(i));
    
    //Create links
    for (int i=0; i<(int)nodes.size(); i++)
    {
        //Compute KNN
        Point2D node = nodes.at(i);
        Point2D node_knn[KNN_MAX];
        for(int j=0; j<knn; j++)
            node_knn[j]=Point2D(POINT_COORD_MAX, POINT_COORD_MAX);
        Knn(node, nodes, knn, node_knn, this->r);

        //Check for collision with objects
        //TODO

        //Insert link in the vector
        for (int j = 0; j < knn; j++)
            if(node_knn[j].x!=POINT_COORD_MAX && node_knn[j].y!=POINT_COORD_MAX && !link_exists(node, node_knn[j], links))
                links.push_back(Segment(Point2D(node.x,node.y),Point2D(node_knn[j].x, node_knn[j].y)));      
    }

    

    //Create Dubins Curves
    for(auto link: links){
        std::vector<DubinLink> d_links;
        // compute every dubins curve for each M_PI/4 
        double step = M_PI * 0.25;
        for(double th_src = 0; th_src < 2 * M_PI; th_src += step){
            DubinPoint node1(link.node1.x, link.node1.y, th_src);
            for(double th_dst = 0; th_dst < 2 * M_PI; th_dst += step){
                DubinPoint node2(link.node2.x, link.node2.y, th_dst);
                // compute dubins from node1 to node2
                auto curves = dubin_curves(node1, node2);
                for(auto curve: curves){
                    bool inter = false;
                    for(auto ob: r.get_obstacles()){
                        if(intersect(curve, ob)){
                            inter = true;
                            break;
                        }
                    }
                    // if the curve doesn't collide with any obstacle, keep this as best curve for the pair (src, dst)
                    if(!inter){
                        d_links.push_back(DubinLink(node1, node2, curve));
                        break;
                    }
                }

                // compute dubins from node2 to node1
                curves = dubin_curves(node2, node1);
                for(auto curve: curves){
                    bool inter = false;
                    for(auto ob: r.get_obstacles()){
                        if(intersect(curve, ob)){
                            inter = true;
                            break;
                        }
                    }
                    if(!inter){
                        d_links.push_back(DubinLink(node2, node1, curve));
                        break;
                    }
                }
            }
        }
        curves.push_back(d_links);
    }

    return true;
}

std::string RoadMap::getJson()
{
    std::string json="";
    json += "{\"roadmap\":";
    json += "{\"nodes\":[";
    for(int i=0; i<(int)nodes.size();i++)
    {
        json+="{\"x\":"+std::to_string(nodes[i].x)+",\"y\":"+std::to_string(nodes[i].y)+"}";
        if(i+1<(int)nodes.size()) json+=",";
    }
    json+="],\"links\":[";
    for(int i=0; i<(int)links.size();i++)
    {
        json+="{\"node1\":{\"x\":"+std::to_string(links[i].node1.x)+",\"y\":"+std::to_string(links[i].node1.y)+"},\"node2\":{\"x\":"+std::to_string(links[i].node2.x)+",\"y\":"+std::to_string(links[i].node2.y)+"}}";
        if(i+1<(int)links.size()) json+=",";
    }
    json+="],\"obstacles\":[";
    for(int i=0; i<r.getNumObstacles();i++)
    {
        json+="{\"normal\":[";
        for(int j=0;j<r.getObstacle(i).get_size();j++)
        {
            json+="{\"x\":"+std::to_string(r.getObstacle(i).get_v(j).x)+",\"y\":"+std::to_string(r.getObstacle(i).get_v(j).y)+"}";
            if(j+1<r.getObstacle(i).get_size()) json+=",";
        }
        json+="],\"inflated\":[";
        for(int j=0;j<r.getInflatedObstacle(i).get_size();j++)
        {
            json+="{\"x\":"+std::to_string(r.getInflatedObstacle(i).get_v(j).x)+",\"y\":"+std::to_string(r.getInflatedObstacle(i).get_v(j).y)+"}";
            if(j+1<r.getInflatedObstacle(i).get_size()) json+=",";
        }
        json+="]}";
        if(i+1<r.getNumObstacles()) json+=",";
    }
    json+="]}}";
    return json;
}

void RoadMap::getAttachedNodes(Point2D node, std::vector<Point2D> *attached_links)
{
    attached_links->clear();
    for(int i=0; i<(int)links.size(); i++)
    {
        if(node==links.at(i).node1)
            attached_links->push_back(links.at(i).node2);
        else if(node==links.at(i).node2)
            attached_links->push_back(links.at(i).node1);
    }
}

/*DubinLink RoadMap::get_dubin_link(Segment link, double th1, double th2)
{
    for(int i=0; i<links.size(); i++)
    {
        if((links[i].node1 == link.node1 && links[i].node2 == link.node2) || (links[i].node1 == link.node2 && links[i].node2 == link.node1))
        {
            for(auto dubin_link: curves[i])
            {
                if(dubin_link.th1 == th1 && dubin_link.th2 == th2)
            }
        }
    }
}*/

void random_obstacles_side(Room* room, int num_obstacles, const int max_side)
{
  Point2D* centers = new Point2D[num_obstacles];
  for(int i=0; i<num_obstacles; i++)
  {
    Polygon o;
    bool check;
    do{
      centers[i].x = ((rand()%(room->getWidth()*100-max_side))+max_side/2)/100.0;
      centers[i].y = ((rand()%(room->getHeight()*100-max_side))+max_side/2)/100.0;
      check=true;
      for(int j=0;j<i;j++)
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
    room->addObstacle(o);
  }
}

void random_obstacles_vertexes(Room* room, int num_obstacles, int vertexes_n){
    random_device rd;

    mt19937 gen(rd());

    uniform_real_distribution<double> x_dis(0, room->getWidth());
    uniform_real_distribution<double> y_dis(0, room->getHeight());
    
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
        room->addObstacle(obstacle);
    }
}