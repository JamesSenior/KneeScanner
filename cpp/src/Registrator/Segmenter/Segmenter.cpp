#include "Registrator/Segmenter/Segmenter.h"
#include "common/kd_tree.h"
#include <functional>
#include <queue>


//constructors:
Segmenter::Segmenter(Matrix3D points, std::function<void(StatusEvent)> callback)
{
    m_points = points;
    m_callback = callback;
}
Segmenter::~Segmenter(){}




//helper functions:
Matrix3D removePoints(Matrix3D points, std::vector<int> deadPoints)
{
    //remove points and return
    std::vector<bool> remove(points.rows(), false);

    for(int index : deadPoints)
    {
        remove[index] = true;
    }

    int remaining = 0;

    for(bool r : remove)
    {
        if(!r)
            remaining++;
    }


    Matrix3D filtered(remaining, 3);

    int row = 0;

    for(int i = 0; i < points.rows(); i++)
    {
        if(!remove[i])
        {
            filtered.row(row) = points.row(i);
            row++;
        }
    }

    return filtered;
}







//Methods//


//Ransac//
//remove floor

void Segmenter::ransac(float threshold, int iterations)
{
    //the idea is to pick 3 random points. these define a random plane. count how many points are on this plane. repeat. the one with the most is the floor


    //call back
    StatusEvent event;
    event.component = Component::Registrator;
    event.subcomponent = Component::Segmentor;
    event.algorithm = Algorithm::RANSAC;
    event.level = LogLevel::NewUpdate;
    m_callback(event);


    int maxCount = 0;
    std::vector<int> maxInliers;

    for(int i=0;i<iterations;i++)
    {
        //pick 3 random points
        int index1 = rand() % m_points.rows();
        int index2 = rand() % m_points.rows();
        int index3 = rand() % m_points.rows();

        Eigen::Vector3f p1 = m_points.row(index1);
        Eigen::Vector3f p2 = m_points.row(index2);
        Eigen::Vector3f p3 = m_points.row(index3);

        //compute 2 vectors or lines which lie on the plane
        Eigen::Vector3f v1 = p2 - p1;
        Eigen::Vector3f v2 = p3 - p1;

        //compute the normal
        Eigen::Vector3f normal = v1.cross(v2);

        //check the 3 random points are not in the same line
        if (normal.norm() < 1e-6f)
        {
            // Bad sample, choose another three points
            continue;
        }
        else
        {
            normal.normalize();
        }

        //compute the distance from the plane (normal distance) of every point
        Eigen::VectorXf distances = ((m_points.rowwise() - p1.transpose()) * normal).cwiseAbs();

        //find which points are close to the plane
        std::vector<int> inliers;

        for (int i = 0; i < distances.size(); i++)
        {
            if (distances(i) < threshold)
            {
                inliers.push_back(i);
            }
        }

        //count and keep if it has largest number of points
        if(inliers.size() > maxCount)
        {
            maxCount = inliers.size();
            maxInliers = inliers;
        }


        //call back
        StatusEvent event;
        event.component = Component::Registrator;
        event.subcomponent = Component::Segmentor;
        event.algorithm = Algorithm::RANSAC;
        event.level = LogLevel::Update;
        event.iteration = i;
        event.max_iterations = iterations;
        event.value = inliers.size();
        m_callback(event);

    }


    //remove maxInlyers from points and return
    m_points = removePoints(m_points, maxInliers);

}







//Euclidean Cluster//
//remove points in the void not connected to main cluster

void Segmenter::euclidean_cluster(float threshold)
{

    //call back
    StatusEvent event;
    event.component = Component::Registrator;
    event.subcomponent = Component::Segmentor;
    event.algorithm = Algorithm::EuclideanCluster;
    event.level = LogLevel::NewUpdate;
    m_callback(event);


    //build kd tree
    PointCloudAdaptor adaptor(m_points);
    KDTree tree(3, adaptor);
    tree.buildIndex();

    //create viseted array
    std::vector<bool> visited(m_points.rows(), false); //keep track of which points have been visited

    //create vector of vector to store all the clusters
    std::vector<std::vector<int>> clusters;

    for(int i = 0; i< m_points.rows(); i++)
    {
        if(visited[i] == true) //point has been visted
        {
            continue; //skip point
        }


        //point is of a new cluster
        //now will do breadth first search of cluster

        std::queue<int> q; //keeps track of unexpanded points
        std::vector<int> cluster; //keeps track of points in the cluster

        //add starting point
        q.push(i);
        visited[i] = true;


        while(!q.empty()) //while point to expand
        {
            //get point from q
            int point = q.front();
            q.pop();

            //add point to cluster
            cluster.push_back(point);

            //find neighbours using kd tree and radius
            float query[3] = {m_points(point,0), m_points(point,1), m_points(point,2)}; //makes an array of the 3 cooridnates of point
            std::vector<nanoflann::ResultItem<uint32_t, float>> matches;
            tree.radiusSearch(query, threshold * threshold, matches); //expects radius squared to that is why its r*r

            //loop through found negoubours
            for (const auto& match : matches)
            {
                int neighbour = match.first; //this gets the int of the found negoubour index

                if (!visited[neighbour]) //if it is not already visited
                {
                    visited[neighbour] = true; //set visted to true
                    q.push(neighbour); //add it to q
                }
            }

        }

        //save cluster
        clusters.push_back(cluster);


        //call back
        StatusEvent event;
        event.component = Component::Registrator;
        event.subcomponent = Component::Segmentor;
        event.algorithm = Algorithm::EuclideanCluster;
        event.level = LogLevel::Update;
        event.iteration = std::count(visited.begin(), visited.end(), true); //how many have been visited
        event.max_iterations = visited.size();
        m_callback(event);

    }


    //now all clusters are in clusters
    //combine all which are not part of largest cluster which is probably the main part of the scan
    //find largest cluster
    if (clusters.empty()){ //checks its not empty
        return;
    }

    size_t largest = 0;
    for (size_t i = 1; i < clusters.size(); i++)
    {
        if (clusters[i].size() > clusters[largest].size())
        {
            largest = i;
        }
    }

    //combine all other clusters
    std::vector<int> outliers;
    for (size_t i = 0; i < clusters.size(); i++)
    {
        if (i == largest)
            continue;

        outliers.insert(outliers.end(), clusters[i].begin(), clusters[i].end());
    }


    //delete all which are not part of the largest cluster and return
    m_points = removePoints(m_points, outliers);

}