#include "Aligner.h"
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <limits>



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
            return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1) ^ (std::hash<int>()(key.z) << 2);
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






float initialize_sigma2(const Matrix3D& X, const Matrix3D& Y)
{
    int N = X.rows();
    int M = Y.rows();
    int D = X.cols();

    float sum = 0.0f;

    for (int m = 0; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            Eigen::Vector3f diff = X.row(n) - Y.row(m);
            sum += diff.squaredNorm();
        }
    }

    return sum / (D * M * N);
}







//make matrix G: a matrix of size NxN which hold how close each point influences each other point in the source matirx
Eigen::MatrixXf gaussian_kernel(const Matrix3D& X, float beta)
{
    int N = int(X.rows());

    Eigen::MatrixXf G(N, N);

    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            Eigen::Vector3f diff = X.row(i) - X.row(j);

            float squared_distance = diff.squaredNorm();

            G(i, j) = std::exp(-squared_distance / (2 * beta * beta));
        }
    }

    return G;
}






//This is the E-Step. it takes the deformed source and matches a probability of it to every point in the target cloud
//It also makes PX which is a weighted matrix which multiplies the target by the probability cloud
void expectation(
    const Matrix3D& target,
    const Matrix3D& transformed_source,
    float sigma2,
    float w,
    Eigen::MatrixXf& P,
    Eigen::VectorXf& Pt1,
    Eigen::VectorXf& P1,
    float& Np,
    Matrix3D& PX)
{
    int N = target.rows();            // target points
    int M = transformed_source.rows(); // source points
    int D = target.cols();

    // Step 1: Calculate squared distances
    for (int m = 0; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            Eigen::Vector3f diff = transformed_source.row(m) - target.row(n);

            float distance_squared = diff.squaredNorm();

            // Step 2: Gaussian probability
            P(m, n) = std::exp(-distance_squared / (2 * sigma2));
        }
    }


    // Step 3: Outlier term
    float c = std::pow(2 * 3.14159265359f * sigma2, D / 2.0f)
              * w / (1.0f - w)
              * M / float(N);


    // Step 4: Normalise probabilities
    for (int n = 0; n < N; n++)
    {
        float denominator = 0.0f;

        for (int m = 0; m < M; m++)
        {
            denominator += P(m, n);
        }

        denominator += c;

        // Prevent division by zero
        if (denominator < std::numeric_limits<float>::epsilon())
            denominator = std::numeric_limits<float>::epsilon();


        for (int m = 0; m < M; m++)
        {
            P(m, n) /= denominator;
        }
    }


    // Step 5: Calculate P1, Pt1, Np
    //P1 = row sums of P
    // Pt1 collumn sums of P
    //Np = total probability
    //PX = weighted target positions
    P1.setZero();
    Pt1.setZero();

    for (int m = 0; m < M; m++)
    {
        for (int n = 0; n < N; n++)
        {
            P1(m) += P(m, n);
            Pt1(n) += P(m, n);
        }
    }

    Np = P1.sum();


    // Step 6: Calculate PX = P * X
    PX = P * target;
}








//This is the M-Step. it calculates the deformation of the source points. it outputs W which are deformation weights
void update_transform(
    const Eigen::MatrixXf& G,
    const Matrix3D& source,
    const Eigen::VectorXf& P1,
    const Matrix3D& PX,
    float alpha,
    float sigma2,
    Matrix3D& W)
{
    int M = source.rows();

    // Create diagonal matrix from P1
    Eigen::MatrixXf diagP1 = P1.asDiagonal();

    // Create identity matrix
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(M, M);

    // A = diag(P1)G + alpha*sigma2*I
    Eigen::MatrixXf A = diagP1 * G
                      + alpha * sigma2 * I;

    // B = PX - diag(P1)Y
    Matrix3D B = (PX - diagP1 * source).eval();

    // Solve AW = B
    W = (A.colPivHouseholderQr().solve(B)).eval();
}








void update_variance(
    const Matrix3D& target,
    const Matrix3D& transformed_source,
    const Eigen::VectorXf& Pt1,
    const Eigen::VectorXf& P1,
    const Matrix3D& PX,
    float Np,
    int D,
    float tolerance,
    float& sigma2,
    float& diff)
{
    float previous_sigma2 = sigma2;


    // xPx = Pt1^T * sum(X^2)
    float xPx = 0.0f;

    for (int n = 0; n < target.rows(); n++)
    {
        xPx += Pt1(n) * target.row(n).squaredNorm();
    }


    // yPy = P1^T * sum(TY^2)
    float yPy = 0.0f;

    for (int m = 0; m < transformed_source.rows(); m++)
    {
        yPy += P1(m) * transformed_source.row(m).squaredNorm();
    }


    // trPXY = sum(TY * PX)
    float trPXY = (transformed_source.array() * PX.array()).sum();


    // Update sigma²
    sigma2 = (xPx - 2.0f * trPXY + yPy) / (Np * D);


    // Prevent negative variance
    if (sigma2 <= 0)
    {
        sigma2 = tolerance / 10.0f;
    }


    // Used for convergence check
    diff = std::abs(sigma2 - previous_sigma2);
}









//CPD alignment//
//coheent point drift (non-rigid transform)
void Aligner::cpd_alignment(float alpha, float beta, float w, int max_iterations, float tolarence, float downsample)
{
    
    //call back that algerithum has started
    StatusEvent event;
    event.component = Component::Registrator;
    event.subcomponent = Component::Aligner;
    event.algorithm = Algorithm::CPD;
    event.level = LogLevel::NewUpdate;
    m_callback(event);
    
    
    
    
    //STEP 1: Represent point clouds
    Matrix3D target = voxelDownsample(m_scan, downsample);
    Matrix3D source = voxelDownsample(m_master, downsample);
    
    
    //STEP 2: Initilise CPD Parameters
    
    //variables:
    Matrix3D transformed_source = source; //this is the source that is changed every iteration
    
    int target_rows = int(target.rows()); //variables to hold the shapes of matrixes
    int source_rows = int(source.rows());
    
    float diff = std::numeric_limits<float>::infinity(); //used for convergance check
    
    //parameter initilisation
    float sigma2 = initialize_sigma2(target, source); //initilises parameter sigma
    
    //matrix variables
    // Probability matrix
    Eigen::MatrixXf P = Eigen::MatrixXf::Zero(source_rows, target_rows);

    // Probability sums
    Eigen::VectorXf P1 = Eigen::VectorXf::Zero(source_rows);
    Eigen::VectorXf Pt1 = Eigen::VectorXf::Zero(target_rows);

    // Weighted target positions
    Matrix3D PX = Matrix3D::Zero(source_rows, 3);

    float Np = 0;
    
    //weighted matrix
    Matrix3D W = Matrix3D::Zero(source_rows, 3);
    
    
    
    
    
    //STEP 3: Create Gaussian Kernal Matrix
    Eigen::MatrixXf G = gaussian_kernel(source, beta);
    
    
    //STEP 4: Starrt EM optimisation loop
    for(int iteration = 0; iteration < max_iterations; iteration++)
    {
        //STEP 4.1: E-Step
        expectation(target, transformed_source, sigma2, w, P, Pt1, P1, Np, PX);
        
        //STEP 4.2: M-Step
        update_transform(G, source, P1, PX, alpha, sigma2, W);
        
        //STEP 4.3: Apply Deformation
        transformed_source = source + G * W;
        
        //STEP 4.4: Update variance
        update_variance(target, transformed_source, Pt1, P1, PX, Np, 3, tolarence, sigma2, diff);
        
        //STEP 4.5: Check convergance
        if(diff<tolarence)
        {
            //break;
        }
        
        //callback
        StatusEvent event;
        event.component = Component::Registrator;
        event.subcomponent = Component::Aligner;
        event.algorithm = Algorithm::CPD;
        event.level = LogLevel::Update;
        event.iteration = iteration;
        event.max_iterations = max_iterations;
        event.value = diff;
        
        m_callback(event);
    }
    
    
    //STEP 5: return results
    m_master = transformed_source;
    
}

