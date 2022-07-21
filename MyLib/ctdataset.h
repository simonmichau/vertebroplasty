#ifndef CTDATASET_H
#define CTDATASET_H

#include "MyLib_global.h"
#include "icpalgo.h"
#include <vector>

typedef struct {
    int x;
    int y;
    int z;
} Voxel;

/**
 * @brief Functions and Infrastructure to work with datasets from CT scans
 */
class MYLIB_EXPORT CTDataset
{
public:
    /// Constructor for CTDataset
    CTDataset();
    /// Destructor for CTDataset
    ~CTDataset();

    /// List of marker centroids
    std::vector<Voxel> markerCentroids;
    /// Array containing information which voxels have been visited already
    bool* visited_voxel;

    short* crosssectionImageData;

    /// Returns the m_pImageData
    short* data();
    /// Returns the m_pDepthBuffer
    short* depthbuffer();
    /// Returns the m_pRegionData
    short* region();

    /// Loads an image file
    int load(QString imagePath);

    /// Windowing function
    static int windowing(int HU_value, int startValue, int windowWidth, int &grayValue);

    /// Calculates the depth buffer for a given threshold
    int calculateDepthBuffer(const int& iThreshold, short* imageData);
    /// Renders a 3D shaded buffer from a given depth buffer
    int renderDepthBuffer(short* shadedBuffer);

    /// Performs region growing
    int regionGrowing(Voxel seed, int threshold, std::vector <Voxel>& iRegion);
    /// Determines registration marker regions
    void getRegistrationMarkers(int threshold);

    void registerMarkers();
    void reconstructLayer(Voxel pos, Voxel axis, Voxel xdir);
    void reconstructLayer_world(Eigen::Vector3d worldPos, Eigen::Vector3d worldAxis, Voxel xdir);

private:
    /// A representation of the original multilayer image
    short* m_pImageData;
    /// A depth map for a multilayer image
    short* m_pDepthBuffer;
    /// 3D object created during region growing
    short* m_pRegionData;

    // Size constants
    int WIDTH = 400;
    int HEIGHT = 400;
    int LAYERS = 400;

    /// Rotates m_pImageData by 90 degrees
    void rotateImage();

    /// Saves centroid of a given region to markerCentroids
    void getCentroid(std::vector<Voxel> region);

    /// Gets height of region
    int getHeight(std::vector <Voxel> region);
    /// Gets width of region
    int getWidth(std::vector <Voxel> region);
    /// Gets maximum x-value of region
    int getMaxX(std::vector <Voxel> region);
    /// Gets minimum x-value of region
    int getMinX(std::vector <Voxel> region);

    ///speichert die Gesamttransformationsmatrix
    Eigen::Matrix4d resultMatrixInverse;
    Eigen::Matrix3d resultMatrixRotation;

};

#endif // CTDATASET_H
