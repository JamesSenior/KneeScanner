#include "Assembler.h"
#include <iostream>
#include <string>
#include <vector>
#include "io/PLYFile.h"

//constructor
Assembler::Assembler(Matrix3D leftLeg, Matrix3D rightLeg, Matrix3D kneeling, std::map<std::string, Eigen::Vector3f> leftLandmarks, std::map<std::string, Eigen::Vector3f> rightLandmarks, std::map<std::string, Eigen::Vector3f> kneelingLandmarks, std::function<void(StatusEvent)> callback)
{
    m_left_leg = leftLeg;
    m_right_leg = rightLeg;
    m_kneeling = kneeling;
    
    m_left_landmarks = leftLandmarks;
    m_right_landmarks = rightLandmarks;
    m_kneeling_landmarks = kneelingLandmarks;
    
    m_callback = callback;
}




//helper functions:

//this method takes a point cloud and 3 points to make a plane and returns one side of it depending on the preserve
Matrix3D planeCut(Matrix3D cloud, Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3, Eigen::Vector3f preserve){
    
    // 1. Create two vectors lying on the plane
    Eigen::Vector3f v1 = p2 - p1;
    Eigen::Vector3f v2 = p3 - p1;
    
    // 2. Find the vector perpendicular to the plane
    Eigen::Vector3f normal = v1.cross(v2);
    
    // 3. Make the normal a unit vector (make size 1.0)
    normal.normalize();
    
    // 4. Make sure the normal points in the direction we want to keep
    if (normal.dot(preserve) < 0)
    {
        normal = -normal;
    }
    
    // 5. Create an empty cloud for the result
    Matrix3D outputCloud;
    
    

    // 6. Check every point in the cloud
    for (int i = 0; i < cloud.rows(); ++i)
    {
        Eigen::Vector3f point = cloud.row(i);

        // Vector from the plane to this point
        Eigen::Vector3f pointVector = point - p1;

        // Determine which side of the plane the point is on
        float side = pointVector.dot(normal);

        // Keep points on the desired side
        if (side >= 0)
        {
            outputCloud.conservativeResize(outputCloud.rows() + 1, 3);
            outputCloud.row(outputCloud.rows() - 1) = point;
        }
    }
    
    return outputCloud;
    
}













//orientator helper functions:



// ============================================================
// STEP 1: Match landmarks between two scans by NAME.
//
// Why: to compute an alignment we need pairs of points that
// represent the SAME physical spot on the knee — one from the
// source scan (the one we're moving) and one from the target
// scan (the one that stays fixed). Matching by name guarantees
// we're comparing the right point to the right point.
//
// Landmarks that only exist in one of the two maps are ignored,
// since there's nothing to pair them with.
// ============================================================
void matchLandmarks(const std::map<std::string, Eigen::Vector3f>& source,
                     const std::map<std::string, Eigen::Vector3f>& target,
                     Matrix3D& outSource,   // filled with matched source points
                     Matrix3D& outTarget,   // filled with matched target points (same order)
                     std::vector<std::string> names)
{
    std::vector<Eigen::Vector3f> srcPts, tgtPts;

    // Loop over every landmark in the source scan
    for (const auto& [name, srcPoint] : source) {

            // Check 1: does this landmark also exist in the target map?
            auto it = target.find(name);
            if (it == target.end()) {
                continue; // no match in target -> skip
            }

            // Check 2: is this landmark name one we actually want to use?
            // std::find searches allowedNames for "name". If it reaches
            // the end without finding it, the name isn't in the list.
            bool isAllowed = std::find(names.begin(), names.end(), name) != names.end();
            if (!isAllowed) {
                continue; // not in our whitelist -> skip
            }

            // Both checks passed -> this is a usable, approved pair
            srcPts.push_back(srcPoint);
            tgtPts.push_back(it->second);
        }

    // Convert the matched point lists into Nx3 matrices,
    // which is the format the alignment function expects.
    outSource.resize(srcPts.size(), 3);
    outTarget.resize(tgtPts.size(), 3);
    for (size_t i = 0; i < srcPts.size(); ++i) {
        outSource.row(i) = srcPts[i];
        outTarget.row(i) = tgtPts[i];
    }
}


// ============================================================
// A rigid transform = rotation (R) + translation (t).
// "Rigid" means: it can rotate and slide the points around,
// but it can NEVER stretch, shrink, or mirror them.
// That's exactly "rotated and translated but not flipped or scaled".
// ============================================================
struct RigidTransform {
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); // rotation (3x3), starts as "no rotation"
    Eigen::Vector3f t = Eigen::Vector3f::Zero();      // translation (3x1), starts as "no movement"

    // Apply this transform to an entire point cloud (Nx3).
    // For every point p, the new position is: R * p + t
    Matrix3D apply(const Matrix3D& points) const {
        Matrix3D out = points * R.transpose(); // rotate all points at once
        out.rowwise() += t.transpose();        // then shift all points by t
        return out;
    }

    // Apply this transform to a single point (handy for landmarks).
    Eigen::Vector3f apply(const Eigen::Vector3f& point) const {
        return R * point + t;
    }
};


// ============================================================
// STEP 2: Given matched landmark pairs, find the ONE rotation +
// translation that best lines up sourceLandmarks onto targetLandmarks.
//
// "Best" = minimizes the total distance error across all landmark
// pairs after moving the source points.
//
// This uses Eigen's built-in Umeyama method, which solves exactly
// this problem and is guaranteed to return a true rotation
// (no mirroring/flipping), as long as with_scaling = false.
// ============================================================
RigidTransform computeRigidAlignment(const Matrix3D& sourceLandmarks,
                                      const Matrix3D& targetLandmarks)
{
    // Eigen's umeyama function wants points as COLUMNS (3xN),
    // but we store points as ROWS (Nx3), so we transpose here.
    Eigen::Matrix3Xf src = sourceLandmarks.transpose();
    Eigen::Matrix3Xf tgt = targetLandmarks.transpose();

    // Solve for the best-fit rotation + translation.
    // with_scaling = false is important: it forbids resizing the
    // points, which is what you asked for (no scaling).
    Eigen::Matrix4f T = Eigen::umeyama(src, tgt, /*with_scaling=*/false);

    // T is a 4x4 matrix that bundles R and t together like this:
    //   [ R  R  R  t ]
    //   [ R  R  R  t ]
    //   [ R  R  R  t ]
    //   [ 0  0  0  1 ]
    // so we just pull out the pieces we want.
    RigidTransform result;
    result.R = T.block<3,3>(0,0); // top-left 3x3 = rotation
    result.t = T.block<3,1>(0,3); // top-right 3x1 = translation
    return result;
}



Matrix3D orientate(Matrix3D source, std::map<std::string, Eigen::Vector3f> sourceLandmarks, std::map<std::string, Eigen::Vector3f> targetLandmarks, std::vector<std::string> names)
{
    // --- Step 1a: find matching landmark pairs by name ---
    Matrix3D srcPts, tgtPts; //these will hold the landmark points which are pairs
    matchLandmarks(sourceLandmarks, targetLandmarks, srcPts, tgtPts, names);

    // --- Step 2b: compute the rotation + translation that best aligns them ---
    RigidTransform xform = computeRigidAlignment(srcPts, tgtPts);

    // --- Step 3c: apply that SAME transform to the entire leg cloud ---
    // (the kneeling cloud/landmarks are never touched — only the leg scan moves)
    Matrix3D legCloudAligned = xform.apply(source);
    
    return legCloudAligned;
}




//combine helper funcs:
Matrix3D combineScans(const std::vector<Matrix3D>& scans)
{
    int totalRows = 0;

    for (const auto& scan : scans)
    {
        totalRows += scan.rows();
    }

    Matrix3D combined(totalRows, 3);

    int currentRow = 0;

    for (const auto& scan : scans)
    {
        combined.block(currentRow, 0, scan.rows(), 3) = scan;
        currentRow += scan.rows();
    }

    return combined;
}







//methods:

void Assembler::combine()
{
    //callback
    StatusEvent event;
    event.component = Component::Aligner;
    event.algorithm = Algorithm::None;
    event.level = LogLevel::NewUpdate;
    m_callback(event);
    
    
    //Step 1: pane cut
    //we now have 2 parts per leg scan: shin and foot
    
    std::vector<std::string> ankleCutLeft = {"spatsLeft", "achillesLeft", "ankleOutsideLeft"};
    std::vector<std::string> ankleCutRight = {"spatsRight", "achillesRight", "ankleOutsideRight"};
    std::vector<std::string> kneeCutLeft = {"creaseOutsideLeft", "creaseInsideLeft", "calfLeft"};
    std::vector<std::string> kneeCutRight = {"creaseOutsideRight", "creaseInsideRight", "calfRight"};
    
    std::vector<std::string> footPointsLeft = {"achillesLeft", "ankleOutsideLeft", "bigLeft", "smallLeft", "ankleInsideLeft", "heelLeft"};
    std::vector<std::string> shinPointsLeft = {"creaseOutsideLeft", "creaseInsideLeft", "calfLeft", "achillesLeft", "ankleOutsideLeft", "kneeLeft"};
    std::vector<std::string> footPointsRight = {"achillesRight", "ankleOutsideRight", "bigRight", "smallRight", "ankleInsideRight", "heelRight"};
    std::vector<std::string> shinPointsRight = {"creaseOutsideRight", "creaseInsideRight", "calfRight", "achillesRight", "ankleOutsideRight", "kneeRight"};
    
    //feet:
    Matrix3D Lfoot = planeCut(m_left_leg, m_left_landmarks[ankleCutLeft[0]], m_left_landmarks[ankleCutLeft[1]], m_left_landmarks[ankleCutLeft[2]], m_left_landmarks[footPointsLeft[2]]);
    Matrix3D Rfoot = planeCut(m_right_leg, m_right_landmarks[ankleCutRight[0]], m_right_landmarks[ankleCutRight[1]], m_right_landmarks[ankleCutRight[2]], m_right_landmarks[footPointsRight[2]]);
    
    //left shin (needs ankle and knee cut)
    Matrix3D Lshin = planeCut(m_left_leg, m_left_landmarks[kneeCutLeft[0]], m_left_landmarks[kneeCutLeft[1]], m_left_landmarks[kneeCutLeft[2]], m_left_landmarks[shinPointsLeft[3]]);
    Lshin = planeCut(Lshin, m_left_landmarks[ankleCutLeft[0]], m_left_landmarks[ankleCutLeft[1]], m_left_landmarks[ankleCutLeft[2]], m_left_landmarks[shinPointsLeft[2]]);
    
    //right shin (needs ankle and knee cut)
    Matrix3D Rshin = planeCut(m_right_leg, m_right_landmarks[kneeCutRight[0]], m_right_landmarks[kneeCutRight[1]], m_right_landmarks[kneeCutRight[2]], m_right_landmarks[shinPointsRight[3]]);
    Rshin = planeCut(Rshin, m_right_landmarks[ankleCutRight[0]], m_right_landmarks[ankleCutRight[1]], m_right_landmarks[ankleCutRight[2]], m_right_landmarks[shinPointsRight[2]]);
    
    
    
    //Step2: orientate
    Lfoot = orientate(Lfoot, m_left_landmarks, m_kneeling_landmarks, footPointsLeft);
    Rfoot = orientate(Rfoot, m_right_landmarks, m_kneeling_landmarks, footPointsRight);
    
    Lshin = orientate(Lshin, m_left_landmarks, m_kneeling_landmarks, shinPointsLeft);
    Rshin = orientate(Rshin, m_right_landmarks, m_kneeling_landmarks, shinPointsRight);
    
    
    //Step 3: align (ICP each leg part to kneeling)
    
    
    //Step3: prune & combine (remove parts of the kneeling scan which is now replaced by leg scans and combineto single scan)
    //step 3a: combine leg scans
    combined = combineScans({Lfoot, Rfoot, Lshin, Rshin});
    
    
    //step 3b: prune knee scan
    
    //step 3c: combine knee scan
    combined = combineScans({combined, m_kneeling});
    
    
    //delete:
    writeToPLY(Lfoot, "LFoot.ply");
    writeToPLY(Lshin, "LShin.ply");
    
    writeToPLY(Rfoot, "RFoot.ply");
    writeToPLY(Rshin, "RShin.ply");
    
    
}




void Assembler::tessellate()
{
    if(combined.rows() == 0){
        std::cout << "No 'combined' cloud to work on" << std::endl;
        return;
    }
    
    
}



void Assembler::customize()
{
    //this edits kneeling and leg scan and combined probably. it will call combined somewhere in there
    combine();
}
