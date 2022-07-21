#include "icpalgo.h"
#include <cmath>
#include <QDebug>

IcpAlgo::IcpAlgo()
{
    tmpTrafo.setIdentity();
    resultMatrix.setIdentity();
}

IcpAlgo::~IcpAlgo()
{

}

/**
 * @brief   zum Einladen der Zielkoordinaten des Regisrierkoerpers in Raumkoordinaten; die aeussersten vier Koordinaten werden zusaetzlich
 *          in die Liste vorregistrierung geladen
 */
void IcpAlgo::init() {
    targetPoints.clear();
    targetPoints.push_back(Eigen::Vector3d(19.918, 2.107, 50.457));
    targetPoints.push_back(Eigen::Vector3d(-31.981, -2.392, 45.372));
    targetPoints.push_back(Eigen::Vector3d(25.889, 0.092, 40.441));
    targetPoints.push_back(Eigen::Vector3d(-25.965, 0.068, 35.431));
    targetPoints.push_back(Eigen::Vector3d(31.856, -2.281, 30.458));
    targetPoints.push_back(Eigen::Vector3d(-19.957, 2.024, 25.449));
    targetPoints.push_back(Eigen::Vector3d(25.850, 0.255, 8.199));
    targetPoints.push_back(Eigen::Vector3d(-19.910, 2.065, 3.090));
    targetPoints.push_back(Eigen::Vector3d(19.874, 2.171, -1.821));
    targetPoints.push_back(Eigen::Vector3d(-25.897, 0.274, -6.923));
    targetPoints.push_back(Eigen::Vector3d(-19.909, 2.140, -24.177));
    targetPoints.push_back(Eigen::Vector3d(31.837, -2.273, -29.132));
    targetPoints.push_back(Eigen::Vector3d(-25.896, 0.280, -34.239));
    targetPoints.push_back(Eigen::Vector3d(25.963, 0.285, -39.131));
    targetPoints.push_back(Eigen::Vector3d(-31.826, -2.096, -44.269));
    targetPoints.push_back(Eigen::Vector3d(20.046, 2.158, -49.122));

    preregistrationTarget.clear();
    preregistrationTarget.push_back(Eigen::Vector3d(20.046, 2.158, -49.122));
    preregistrationTarget.push_back(Eigen::Vector3d(-31.826, -2.096, -44.269));
    preregistrationTarget.push_back(Eigen::Vector3d(-31.981, -2.392, 45.372));
    preregistrationTarget.push_back(Eigen::Vector3d(19.918, 2.107, 50.457));
}

/**
 * @brief fuehrt den Icp Algorithmus aus; dabei wird mit den vier aeussersten Kugeln eine Vorregistrierung durchgefuehrt
 */
void IcpAlgo::calculate() {
    init();
    tmpTrafo.setIdentity();
    resultMatrix.setIdentity();

    for(size_t i = 0; i<sourcePoints.size(); i++)
        std::cout << "xyz: " << sourcePoints[i].x()<< " " << sourcePoints[i].y()<< " " << sourcePoints[i].z() << " " << std::endl;

    //sourcePoints in aufsteigender Reihenfolge nach z Werten sortieren
    std::sort(sourcePoints.begin(), sourcePoints.end(),[](Eigen::Vector3d a,Eigen::Vector3d b){return a(2)<b(2);});

    //die aeussersten SourcePoints entsprechen den aeussersten Regisrierkoerpern
    preregistrationSource ={sourcePoints[0],sourcePoints[1],sourcePoints[sourcePoints.size()-2],
                            sourcePoints[sourcePoints.size()-1]};

    // 1. Vorregistrierung
    tmpTrafo = estimateRigidTransformation3D(preregistrationSource, preregistrationTarget);
    resultMatrix = tmpTrafo*resultMatrix;
    //Transformation aller sourcePoints
    for(size_t i = 0; i<sourcePoints.size(); i++) {
        sourcePoints[i] = tmpTrafo * sourcePoints[i];
        if(i<4) {
            preregistrationSource[i] = tmpTrafo * preregistrationSource[i];
        }
    }
    double rms = calculateRMS(preregistrationSource, preregistrationTarget);
    std::cout << "Vorregistrierung: " << rms << std::endl;

    // itrative closest point search & transformation
    for(int i=0; i<10; i++) {
        // 2. find closest points
        findTargetPoints(sourcePoints);

        // 3. calculate transformation matrix
        tmpTrafo = estimateRigidTransformation3D(sourcePoints, tmpTargetPoints);
        resultMatrix = tmpTrafo*resultMatrix;

        // 4. transform points
        for(size_t i = 0; i<sourcePoints.size(); i++) {
            sourcePoints[i]=tmpTrafo*sourcePoints[i];
        }

        // (5. one could use rms limit to stop iterations)
        rms = calculateRMS(sourcePoints, tmpTargetPoints);
        std::cout << "RMS: " << rms << std::endl;
    }

    std::cout << "Transformationsmatrix " << std::endl;
    std::cout << resultMatrix.matrix().inverse() << std::endl;
}

///
/// \brief                      calculate point-to-point registration
/// \param sourcePoints:        positions in source coordinate system
/// \param targetPoints:        positions in tareget coordinate system
/// \return Eigen::Matrix4d     homogeneous transformation matrix estimating a rigid body transformation
///                             of the source points to the target points
///
Eigen::Matrix4d IcpAlgo::estimateRigidTransformation3D(const std::vector<Eigen::Vector3d> &sourcePoints, const std::vector<Eigen::Vector3d> &targetPoints)
{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;

    // Transform std::vector of voxels to dynamic sized eigen matrix.
    const int m = 3;
    const int n = static_cast<int>(sourcePoints.size());
    MatrixXd X = MatrixXd(m, n);
    MatrixXd Y = MatrixXd(m, n);
    for (int i = 0; i < n; ++i) {
        X.col(i) = sourcePoints[i].head(3);
        Y.col(i) = targetPoints[i].head(3);
    }

    // Subtract mean.
    Eigen::Vector3d mean_X = X.rowwise().mean();
    Eigen::Vector3d mean_Y = Y.rowwise().mean();
    X.colwise() -= mean_X;
    Y.colwise() -= mean_Y;

    // Compute SVD (singular value decomposition) of cross-covariance matrix.
    MatrixXd R_XY = X * Y.adjoint();
    Eigen::JacobiSVD<MatrixXd> svd(R_XY, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Compute estimate of the rotation matrix:
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().adjoint();

    // Assure a right-handed coordinate system:
    if (R.determinant() < 0) {
        R = svd.matrixV() * Eigen::Vector3d(1, 1, -1).asDiagonal() * svd.matrixU().transpose();
    }

    // Construct homogeneous transformation matrix.
    Eigen::Matrix4d transformationMatrix;
    transformationMatrix.block(0, 0, 3, 3) = R;
    transformationMatrix.block(0, 3, 3, 1) = mean_Y - R * mean_X;
    transformationMatrix.block(3, 0, 1, 3) = Eigen::RowVector3d::Zero();
    transformationMatrix(3, 3) = 1.0;

    return transformationMatrix;
}

/// \brief                      berechnet den root mean square Error (RMS) der Abstaende von sourcePoints zu targetPoints
/// \param sourcePoints
/// \param targetPoints
/// \return distance            Quadratische Mittel der Abstaende
///
double IcpAlgo::calculateRMS(std::vector<Eigen::Vector3d> &sourcePoints, std::vector<Eigen::Vector3d> &targetPoints)
{
    double rms = 0.0;
    for (std::size_t i = 0; i<sourcePoints.size(); i++) {
        Eigen::Vector3d temp_target = targetPoints[i];
        Eigen::Vector3d temp_source = sourcePoints[i];
        rms += pow((temp_target.x()-temp_source.x()), 2)+pow((temp_target.y()-temp_source.y()),2)+pow((temp_target.z()-temp_source.z()),2);
    }
    rms = sqrt(rms);
    rms/=sourcePoints.size();
    return rms;
}

/**
 * @brief                   bestimmt zu jedem sourcePoint den passenden TargetPoint mit geringstem Abstand und speicher ihn in der Liste tmpTargetPoints
 * @param sourcePoints
 * @return
 */
void IcpAlgo::findTargetPoints(std::vector<Eigen::Vector3d> &sourcePoints)
{
    // first clean up the tmptargetPoints List
    tmpTargetPoints.clear();

    Eigen::Vector3d point;
    for (unsigned long int i=0; i<sourcePoints.size(); i++){
        point = sourcePoints[i];

        //find closest target point for current source point
        Eigen::Vector3d tmpClosestPoint;
        int min_distance = getDistance(targetPoints[0], point);
        tmpClosestPoint = targetPoints[0];
        for (unsigned long int j = 1; j<targetPoints.size(); j++){
            if (getDistance(targetPoints[j], point) < min_distance){
                min_distance = getDistance(targetPoints[j], point);
                tmpClosestPoint = targetPoints[j];
            }
        }
        // save closest point to tmpTargetPoints
        tmpTargetPoints.push_back(tmpClosestPoint);
    }

}


/**
 * @brief Bestimmt den Abstand zwischen zwei 3D Vektoren und gibt ihn zurück
 * @param a
 * @param b
 * @return
 */
int IcpAlgo::getDistance(Eigen::Vector3d a, Eigen::Vector3d b){
    return sqrt(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2) + pow(a.z() - b.z(), 2));
}

