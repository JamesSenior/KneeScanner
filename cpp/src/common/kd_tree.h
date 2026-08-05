//KD tree helpers:

struct PointCloudAdaptor
{
    const Matrix3D& pts;

    PointCloudAdaptor(const Matrix3D& p) : pts(p) {}

    size_t kdtree_get_point_count() const
    {
        return pts.rows();
    }

    float kdtree_get_pt(size_t idx, size_t dim) const
    {
        return pts(idx, dim);
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const
    {
        return false;
    }
};


using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
    PointCloudAdaptor,
    3>;