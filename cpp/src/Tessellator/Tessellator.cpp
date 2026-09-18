#include "Tessellator.h"
#include <open3d/Open3D.h>


Tessellator::Tessellator(std::function<void(StatusEvent)> callback)
{
    m_callback = callback;
}



//getters and setters
void Tessellator::setPointCloud(const Matrix3D& points)
{
    m_points = points;
}



//helper funcs






//methods
void Tessellator::tessellate(float alpha, float beta)
{
    
    //Step 1: bring point cloud into Open3d datatype

    StatusEvent event;
    event.component = Component::Tessellator;
    event.algorithm = Algorithm::PointCloudConversion;
    event.level = LogLevel::NewUpdate;
    event.message = "Converting scan points for Open3D.";
    m_callback(event);
    
    open3d::geometry::PointCloud pointCloud;

    // Reserve space first so Open3D does not repeatedly resize its list.
    pointCloud.points_.reserve(static_cast<size_t>(m_points.rows()));

    for (Eigen::Index row = 0; row < m_points.rows(); ++row)
    {
        double x = static_cast<double>(m_points(row, 0));
        double y = static_cast<double>(m_points(row, 1));
        double z = static_cast<double>(m_points(row, 2));

        // Ignore broken points instead of allowing them to ruin the mesh.
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        {
            continue;
        }

        pointCloud.points_.emplace_back(x, y, z);
    }
    
    
    
    //Step 2: calculate normals

    event.component = Component::Tessellator;
    event.algorithm = Algorithm::NormalEstimation;
    event.level = LogLevel::NewUpdate;
    event.message = "Calculating surface normals.";
    m_callback(event);
    
    const double normalSearchRadius = 10.0;
    const int maximumNeighbours = 30;

    pointCloud.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(normalSearchRadius, maximumNeighbours));

    // Make nearby normal arrows point in a consistent direction.
    pointCloud.OrientNormalsConsistentTangentPlane(maximumNeighbours);

    // Ensure every normal arrow has a length of exactly 1.
    pointCloud.NormalizeNormals();
    
    
    
    //Step 3: generate mesh (poisson surface)

    event.component = Component::Tessellator;
    event.algorithm = Algorithm::PoissonReconstruction;
    event.level = LogLevel::NewUpdate;
    event.message = "Creating the Poisson surface mesh.";
    m_callback(event);

    const size_t poissonDepth = 7; // Start at 7. Later we can test 8 if the knee detail needs it.

    auto [mesh, densities] = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(pointCloud, poissonDepth);

    if (!mesh || mesh->vertices_.empty())
    {
        throw std::runtime_error("Open3D could not create a surface mesh.");
    }

    // Calculate normals for the new triangle mesh.
    mesh->ComputeVertexNormals();
    
    
    //Step 4: Binary subtracts:
    //first make binding box
    //4a: Boat hull
    //4b: knees
    //4c: offset box

    event.component = Component::Tessellator;
    event.algorithm = Algorithm::MeshTrimming;
    event.level = LogLevel::NewUpdate;
    event.message = "Trimming the mesh.";
    m_callback(event);
    
    
    
    //Step 5: Save file

    event.component = Component::Tessellator;
    event.algorithm = Algorithm::MeshExport;
    event.level = LogLevel::NewUpdate;
    event.message = "Saving the tessellated mesh.";
    m_callback(event);

    const std::string outputPath = "/Users/jamessenior/github/KneeScanner/cpp/resources/output/tessellated.stl";

    const bool saved = open3d::io::WriteTriangleMesh(outputPath, *mesh, false, false, true);

    if (!saved)
    {
        throw std::runtime_error("Could not save the tessellated mesh.");
    }
    
    
    
}
