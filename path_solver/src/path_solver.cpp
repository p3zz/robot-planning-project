#include "path_solver/path_solver.h"
using namespace std;

void sort_knn(double arr1[], std::vector<Point2D> arr2, int k)
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

void Knn(Point2D node, std::vector<Point2D> candidates, int k, std::vector<Point2D> nearest_nodes)
{
    //Init variables
    double* nearest_distances = new double[k];
    for(int i=0; i<k; i++)
    {
        nearest_distances[i]=999999;
        nearest_nodes.push_back(Point2D(0.0,0.0));
    }
    
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

    for(int i=0; i<k; i++)
        cout << nearest_distances[i] << " ("<<nearest_nodes[i].x<<", "<<nearest_nodes[i].y<<") ";    
    cout << endl;
}

void RoadMap::constructRoadMap(int points, int knn)
{
    //Create nodes
    for (int i=0; i<points; i++)
    {
        double x = int(rand()%(r.getWidth()*100)) / 100.0; //cm sensibility
        double y = int(rand()%(r.getHeight()*100)) / 100.0; //cm sensibility
        Point2D node(x, y);
        nodes.push_back(node);
    }
    //Create links
    for (int i=0; i<(int)nodes.size(); i++)
    {
        Point2D node = nodes.at(i);
        std::vector<Point2D> node_knn;
        Knn(node, nodes, knn, node_knn);

        cout<<node.x<<", "<<node.y<<":"<<endl;
        for (int j = 0; j < knn; j++)
        {
            cout<<"\t"<<node_knn[j].x<<", "<<node_knn[j].y<<endl;
        }
        cout << endl;
    }
}