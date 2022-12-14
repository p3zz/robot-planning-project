#include "map/map.hpp"
using namespace std;

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

void Knn(Point2D node, std::vector<Point2D> candidates, int k, Point2D* nearest_nodes)
{
    //Init variables
    double* nearest_distances = new double[k];
    for(int i=0; i<k; i++)
        nearest_distances[i]=999999;
    
    //Test all candidates
    for(int i=0; i<(int)candidates.size(); i++)
    {
        Point2D cand = candidates[i];
        //different nodes
        if(cand.x != node.x && cand.y != node.y)
        {
            //check if a node is better than others (starting from worst node)
            double distance=sqrt(pow(cand.x-node.x,2)+pow(cand.y-node.y,2));
            bool edited=false;
            for(int j=k; j>=0 && !edited; j--)
                if(distance<nearest_distances[j])
                {
                    nearest_distances[j]=distance;
                    nearest_nodes[j]=Point2D(cand.x, cand.y);
                    edited=true;
                }

            if(edited)
                sort_knn(nearest_distances, nearest_nodes, k);
        }
    }
}

bool check_sparse(double x, double y, std::vector<Point2D> nodes, double distance_min)
{
    for (int i = 0; i < (int)nodes.size(); i++) //check distance with other nodes
    {
        double distance=sqrt(pow(x-nodes[i].x,2)+pow(y-nodes[i].y,2));
        if(distance<distance_min) 
            return false;
    }
    return true;    
}

//PRM ROADMAP
bool RoadMap::constructRoadMap(int points, int knn, double k_distance_init, double tms_max)
{
    if(points>KNN_MAX) return false;
    
    //Create nodes
    srand(time(NULL));
    const double k_room_space=(r.getHeight()-ROBOT_CIRCLE*2)*(r.getWidth()-ROBOT_CIRCLE*2)/points;
    const double k_base=std::pow(0.1,1/tms_max); //base to decrease k_distance to 10% at tms_max
    for (int i=0; i<points; i++)
    {
        double x,y,tms,k_distance,distance_pts;
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
            x = int(rand() % int((r.getWidth()-ROBOT_CIRCLE*2)*100)) / 100.0 + ROBOT_CIRCLE; //cm sensibility, consider room border
            y = int(rand() % int((r.getHeight()-ROBOT_CIRCLE*2)*100)) / 100.0 + ROBOT_CIRCLE; //cm sensibility, consider room border
            count--;
        }while(!check_sparse(x, y, nodes, distance_pts)); //check for sparse nodes
        Point2D node(x, y);
        nodes.push_back(node);
    }
    
    //Create links
    for (int i=0; i<(int)nodes.size(); i++)
    {
        //Compute KNN
        Point2D node = nodes.at(i);
        Point2D node_knn[KNN_MAX];
        Knn(node, nodes, knn, node_knn);

        //Check for collision with objects
        //TODO

        //Insert link in the vector
        for (int j = 0; j < knn; j++)
            links.push_back(Link(Point2D(node.x,node.y),Point2D(node_knn[j].x, node_knn[j].y)));        
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
        json+="{\"source\":{\"x\":"+std::to_string(links[i].src.x)+",\"y\":"+std::to_string(links[i].src.y)+"},\"dest\":{\"x\":"+std::to_string(links[i].dest.x)+",\"y\":"+std::to_string(links[i].dest.y)+"}}";
        if(i+1<(int)links.size()) json+=",";
    }
    json+="],\"obstacles\":[";
    for(int i=0; i<r.getNumObstacles();i++)
    {
        json+="[";
        for(int j=0;j<r.getObstacle(i).getNumVertexes();j++)
        {
            json+="{\"x\":"+std::to_string(r.getObstacle(i).getVertex(j).x)+",\"y\":"+std::to_string(r.getObstacle(i).getVertex(j).y)+"}";
            if(j+1<r.getObstacle(i).getNumVertexes()) json+=",";
        }
        json+="]";
        if(i+1<r.getNumObstacles()) json+=",";
    }
    json+="]}}";
    return json;
}