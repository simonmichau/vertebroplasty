#ifndef ICPALGO_H
#define ICPALGO_H

#include "MyLib_global.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <math.h>


/**
* @brief Algorithms to perform point to point-cloud registration by ICP algorithm
*
*
*/
class MYLIB_EXPORT IcpAlgo
{
public:
    IcpAlgo();
    ~IcpAlgo();

    ///Liste der SourcePoints
    std::vector <Eigen::Vector3d> sourcePoints;

    ///Liste der TargetPoints
    std::vector<Eigen::Vector3d> targetPoints;

    ///Liste der TargetPoints fuer die Vorregistrierung
    std::vector<Eigen::Vector3d> preregistrationTarget;

    ///Liste der SourcePoints fuer die Vorregistrierung
    std::vector<Eigen::Vector3d> preregistrationSource;

    ///speichert die Gesamttransformationsmatrix
    Eigen::Transform <double, 3, Eigen::Affine, Eigen::DontAlign> resultMatrix;

    ///fuehrt den ICP-Algorithmus aus
    void calculate();

private:
    ///laedt die Zielkoordinaten in die Target-Liste
    void init();

    ///berechnet die Transormationsmatrix fuer einen einzelnen Icp-Schritt
    Eigen::Matrix4d estimateRigidTransformation3D(const std::vector<Eigen::Vector3d> &sourcePoints, const std::vector<Eigen::Vector3d> &targetPoints);

    ///berechnet den RMS fuer einen Icp-Schritt
    double calculateRMS(std::vector<Eigen::Vector3d> &sourcePoints, std::vector<Eigen::Vector3d> &targetPoints);

    ///sucht fuer jeden sourcePoint den dazugehoerigen targetPoint
    void findTargetPoints(std::vector<Eigen::Vector3d> &sourcePoints);

    ///Liste der targetpoints für jeden Iterationsschritt
    std::vector<Eigen::Vector3d> tmpTargetPoints;

    ///speichert fuer jeden Icp-Schritt die Transformationsmatrix
    Eigen::Transform <double, 3, Eigen::Affine, Eigen::DontAlign> tmpTrafo;

    ///helper function that returns the distance between two points a,b
    int getDistance(Eigen::Vector3d a, Eigen::Vector3d b);
};

#endif // ICPALGO_H
