#include "Aligner.h"
#include "Common/kd_tree.h"

//ICP ALIGNMENT//
//icp (rigid transform)
void Aligner::icp_alignment(int max_iterations, float convergence_threshold)
{

    //make copys to not alter origionals by accident
    Matrix3D source = m_master;
    Matrix3D target = m_scan;

    float lastError; //holds last iterations error to calculate convergance

    
    //callback:
    StatusEvent event;
    event.component = Component::Registrator;
    event.subcomponent = Component::Aligner;
    event.algorithm = Algorithm::ICP;
    event.level = LogLevel::NewUpdate;
    m_callback(event);


    //build kd tree of target
    PointCloudAdaptor adaptor(target);
    KDTree tree(3, adaptor);
    tree.buildIndex();


    for(int i = 0; i<max_iterations; i++) //main icp loop
    {
        //STEP 1: find closest pairs

        std::vector<int> paired_target_indexes; 

        for(int n = 0; n<source.rows();n++) //find nearest neighbour for every source point
        {
            float query[3] = {source(n,0), source(n,1), source(n,2)}; //make querey point
            uint32_t closestIndex; //will store result
            float closestDistance;
            tree.knnSearch(query, 1, &closestIndex, &closestDistance);
            int index = static_cast<int>(closestIndex);
            paired_target_indexes.push_back(index); //add to paired point to vector
        }


        //STEP 2: center selected points by com

        //get new matrix of the points not just the indexes
        Matrix3D paired_target_points(paired_target_indexes.size(), 3);

        for (int k = 0; k < paired_target_indexes.size(); k++)
        {
            paired_target_points.row(k) = target.row(paired_target_indexes[k]);
        }

        //center them
        Eigen::RowVector3f centroidSource = source.colwise().mean();
        Eigen::RowVector3f centroidTarget = paired_target_points.colwise().mean();

        Matrix3D Ps_centered = source.rowwise() - centroidSource;
        Matrix3D Pt_centered = paired_target_points.rowwise() - centroidTarget;


        //STEP 3: #STEP 3: Build covariance matrix H
        Eigen::Matrix3f H_matrix = Ps_centered.transpose() * Pt_centered;


        //STEP 4: compute SVD
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(H_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f U = svd.matrixU();
        Eigen::Matrix3f V = svd.matrixV();

        //STEP 5: compute rotation (R)
        Eigen::Matrix3f R = V * U.transpose();

        if (R.determinant() < 0) //reflection check
        {
            V.col(2) *= -1;
            R = V * U.transpose();
        }


        //STEP 6: compute translation (t)
        Eigen::Vector3f t = centroidTarget.transpose() - R * centroidSource.transpose();

        //STEP 7: apply tranfromation
        source = (source * R.transpose()).rowwise() + t.transpose();

        //STEP 8: check for convergance
        float error = (source - paired_target_points).rowwise().norm().mean();

        
        if(i != 0)//if not the first iteration
        {
            if((lastError - error) < convergence_threshold) //check if error difference is now small
            {
                //break; //exit loop
            }
        }

        lastError = error; //update last error

        //callback:
        StatusEvent event;
        event.component = Component::Registrator;
        event.subcomponent = Component::Aligner;
        event.algorithm = Algorithm::ICP;
        event.level = LogLevel::Update;
        event.iteration = i;
        event.max_iterations = max_iterations;
        event.value = lastError-error;

        m_callback(event);
    }

    m_master = source; //apply to global variable

}





