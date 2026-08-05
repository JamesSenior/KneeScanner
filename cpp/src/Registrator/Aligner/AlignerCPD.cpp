#include "Aligner.h"
#include <iostream>




//helper functions:
Matrix3D voxelDownsample(const Matrix3D& points, float voxelSize)
{
    struct Voxel
    {
        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
        int count = 0;
    };

    struct VoxelKey
    {
        int x, y, z;

        bool operator==(const VoxelKey& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelHash
    {
        std::size_t operator()(const VoxelKey& key) const
        {
            return std::hash<int>()(key.x) ^
                   (std::hash<int>()(key.y) << 1) ^
                   (std::hash<int>()(key.z) << 2);
        }
    };


    std::unordered_map<VoxelKey, Voxel, VoxelHash> voxels;

    // Assign points to voxels
    for (int i = 0; i < points.rows(); i++)
    {
        VoxelKey key{
            static_cast<int>(std::floor(points(i,0) / voxelSize)),
            static_cast<int>(std::floor(points(i,1) / voxelSize)),
            static_cast<int>(std::floor(points(i,2) / voxelSize))
        };

        voxels[key].sum += points.row(i).transpose();
        voxels[key].count++;
    }

    // Create output matrix
    Matrix3D downsampled(voxels.size(), 3);

    int row = 0;
    for (const auto& voxel : voxels)
    {
        downsampled.row(row) =
            (voxel.second.sum / voxel.second.count).transpose();
        row++;
    }

    return downsampled;
}






//CPD alignment//
//coheent point drift (non-rigid transform)
void Aligner::cpd_alignment(float alpha, float beta, int max_iterations, float tolarence, float downsample)
{
    //call back
    StatusEvent event;
    event.component = Component::Registrator;
    event.subcomponent = Component::Aligner;
    event.algorithm = Algorithm::CPD;
    event.level = LogLevel::NewUpdate;
    m_callback(event);


    //STEP 1: downsample
    Matrix3D scan_small = voxelDownsample(m_scan, downsample);
    Matrix3D master_small = voxelDownsample(m_master, downsample);

    //make copy
    Matrix3D master_original = master_small;
    Matrix3D master_current = master_small;
    Matrix3D scan_origional = scan_small;


    //STEP 2: create gausian kernal matrix G or smothing matrix. this is a NxN matrix and links how close each point in the moving cloud is to every other point. 1 is close 0 is far away
    int numMaster = master_original.rows();
    Eigen::MatrixXf G_matrix(numMaster, numMaster);

    for (int i = 0; i < numMaster; i++)
    {
        for (int j = 0; j < numMaster; j++)
        {
            float distanceSquared = (master_original.row(i) - master_original.row(j)).squaredNorm();
            G_matrix(i, j) = std::exp(-distanceSquared / (2.0f * beta * beta));
        }
    }


    //CPD LOOP:

    for(int n = 0; n<max_iterations; n++)
    {

        //STEP 1: probability matrix P. this is now aligning each point in moving cloud with target cloud. but unlike ICP it is not black and white and is probability based similar to the matix above
        int numScan = scan_origional.rows();
        Eigen::MatrixXf P_matrix(numMaster, numScan);

        for (int i = 0; i < numMaster; i++)
        {
            for (int j = 0; j < numScan; j++)
            {
                float distanceSquared = (master_current.row(i) - scan_origional.row(j)).squaredNorm();
                P_matrix(i, j) = std::exp(-distanceSquared / (2.0f * 0.006)); //0.01 is sigma squared
            }
        }

        //Normalise matrix P: we are not done with P as we need to normalise it so all values of a certain point/coloumn/fixedpoint add up to 1
        for (int i = 0; i < P_matrix.cols(); i++)
        {
            float sum = P_matrix.col(i).sum();

            if (sum != 0)
            {
                P_matrix.col(i) /= sum;
            }
        }


        //STEP 2: calculate weighted targets: ie the weighted ceneter of all the points pulling on each moving point.
        Matrix3D weighted_targets = Matrix3D::Zero(numMaster,3);

        for (int i = 0; i < numMaster; i++)
        {
            float prob_sum = P_matrix.row(i).sum();

            if (prob_sum != 0)
            {
                Eigen::Vector3f weighted_sum = Eigen::Vector3f::Zero();

                for (int j = 0; j < numScan; j++)
                {
                    weighted_sum += P_matrix(i,j) * scan_origional.row(j).transpose();
                }

                weighted_targets.row(i) = (weighted_sum / prob_sum).transpose();
            }
        }


        //STEP 3: now we need to calculate another matrix W which is the deformation weights
        Matrix3D displacement = weighted_targets - master_original;

        Eigen::MatrixXf A = G_matrix + alpha * Eigen::MatrixXf::Identity(numMaster, numMaster);

        Matrix3D W_matrix = A.colPivHouseholderQr().solve(displacement);


        
        //STEP 4: Apply transformation
        Matrix3D master_new = master_original + G_matrix * W_matrix;

        //calculates the change
        float change = (master_new - master_current).rowwise().norm().mean();

        //updates ready for new iteration
        master_current = master_new;

        //check for convergence
        if(change < tolarence)
        {
            //break;
        }

        
        //std::cout << tolarence <<std::endl;



        //Callback
        StatusEvent event;
        event.component = Component::Registrator;
        event.subcomponent = Component::Aligner;
        event.algorithm = Algorithm::CPD;
        event.level = LogLevel::Update;
        event.iteration = n;
        event.max_iterations = max_iterations;
        event.value = change;

        m_callback(event);
    }


    //return updated cloud
    m_master = master_current;

}