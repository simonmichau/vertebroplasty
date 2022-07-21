#include "ctdataset.h"
#include "icpalgo.h"
#include <QFile>
#include <cmath>
#include <vector>
#include <QDebug>
#include <QElapsedTimer>
#include "Eigen/Core"
#include "Eigen/Dense"


CTDataset::CTDataset()
{
    m_pImageData = new short[WIDTH*HEIGHT*LAYERS];
    m_pDepthBuffer = new short[WIDTH*HEIGHT];
    m_pRegionData = new short[WIDTH*HEIGHT*LAYERS];
    visited_voxel = new bool[WIDTH*HEIGHT*LAYERS];
    crosssectionImageData = new short[WIDTH*HEIGHT*LAYERS];
}

CTDataset::~CTDataset()
{
    delete[] m_pImageData;
    delete[] m_pDepthBuffer;
    delete[] m_pRegionData;
    delete[] visited_voxel;
    delete[] crosssectionImageData;
}

/**
 * @brief CTDataset::data: Returns the heap of the stored image
 * @return m_pImageData a representation of a multilayer image
 */
short* CTDataset::data()
{
    return m_pImageData;
}

/**
 * @brief CTDataset::depthbuffer: Gets the depth buffer
 * @return m_pDepthBuffer a 2D depth map of a 3D picture as viewed from an axis
 */
short* CTDataset::depthbuffer()
{
    return m_pDepthBuffer;
}

/**
 * @brief CTDataset::region: Gets the growing region
 * @return m_pRegionData a representation of a multilayer image
 */
short* CTDataset::region()
{
    return m_pRegionData;
}

/**
 * @brief CTDataset::load Opens an image file from a given path, checks it and loads it to the heap m_pImageData
 * @param imagePath Path of the image to load
 * @return 0 - no Error occured, 1 - file not found, 2 - file size is inconsistent
 */
int CTDataset::load(QString imagePath)
{
    //QFile Dateiobjekt dataFile erstellen
    QFile dataFile(imagePath);

    //datafile im Lesemodus öffnen
    bool bFileOpen = dataFile.open(QIODevice::ReadOnly);
    if (!bFileOpen){
        return 1; //File not found
    }

    //Überprüfen, ob das Öffnen geklappt hat (iFileSize gleich iNumberBytesRead?) ansonsten Fehlermeldung anzeigen
    int iFileSize = dataFile.size();
    int iNumberBytesRead = dataFile.read((char*)m_pImageData, WIDTH*HEIGHT*LAYERS*sizeof(short));
    if (iFileSize != iNumberBytesRead){
        return 2; //inconsistent File size
    }

    //Inhalt der ausgewählten Datei in Variable imageData einlesen
    dataFile.read((char*)m_pImageData, WIDTH*HEIGHT*LAYERS*sizeof(short));

    // Überprüfen, ob Anzahl eingelesener Bytes der erwarteten Anzahl entsprechen (512*512)
    if (iNumberBytesRead == WIDTH*HEIGHT*LAYERS){
        return 3; //Unexpected count of read bytes
    }
    //dataFile wieder schließen
    dataFile.close();

    //Mirrors x and y-axis to rotate ImageData
    rotateImage();

    return 0; // No Error occured
}

/**
 * @brief CTDataset::rotateImage rotates the m_pImageData by 90 degrees
 */
void CTDataset::rotateImage(){
    short* tmpBuffer = new short[WIDTH*HEIGHT*LAYERS];
    //In einer Doppelschleife über y und x jeweils den zugehörigen index des Speichers berechnen
    for (int l = 0; l < LAYERS; ++l){
        for (int y = 0; y < HEIGHT; ++y) {
            for (int x = 0; x < WIDTH; ++x) {
                tmpBuffer[l*WIDTH*HEIGHT + y*WIDTH + x] = m_pImageData[l*WIDTH*HEIGHT + (HEIGHT-y-1)*WIDTH + (WIDTH-x-1)];
            }
        }
    }
    for (int l = 0; l < LAYERS; ++l){
        for (int y = 0; y < HEIGHT; ++y) {
            for (int x = 0; x < WIDTH; ++x) {
                m_pImageData[l*WIDTH*HEIGHT + y*WIDTH + x] = tmpBuffer[l*WIDTH*HEIGHT + y*WIDTH + x];
            }
        }
    }
    delete [] tmpBuffer;
}

/**
 * @brief CTDataset::windowing Windows 12bit Hounsfield Unit (HU) values to 8bit grayscale values
 * @param HU_value Hounsfield Unit value of a pixel from the 12bit input image
 * @param startValue lower bound of HU values to display
 * @param windowWIDTH WIDTH of the interval of HU values to display
 * @param grayValue the 8bit grayscale value that is calculated from HU_value, startValue and windowWIDTH
 * @return 0 - no Error occured, 1 - HU out of range, 2 - start out of range, 3 - WIDTH out of range
 */
int CTDataset::windowing(int HU_value, int startValue, int windowWIDTH, int &grayValue) {
    if (HU_value < -1024 || 3071 < HU_value){
        return 1; //HU_OUT_OF_RANGE
    }
    else if (HU_value < startValue || startValue < -1042 || 3071 < startValue) {
        grayValue = 0;
        return 2; //START_OUT_OF_RANGE
    }
    else if (HU_value > startValue + windowWIDTH || windowWIDTH < 1 || 4095 < windowWIDTH) {
        grayValue = 255;
        return 3; //WIDTH_OUT_OF_RANGE
    }
    else {
        float scaleFactor = 255.0 / (float) windowWIDTH;
        grayValue = std::roundf((HU_value-startValue) * scaleFactor);
        return 0; //OK
    }
}

/**
 * @brief CTDataset::calculateDepthBuffer: Creates or updates a depth map m_pDepthBuffer that displays all voxels over a certain threshold
 * @param iThreshold the minimum intensity of a voxel to be displayed in the depth buffer
 * @param imageData an arbitrary set of 3D imageData (e.g. m_pImageData)
 * @return 0
 */
int CTDataset::calculateDepthBuffer(const int& iThreshold, short* imageData){
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            for (int l = 0; l < LAYERS; ++l) {
                if (imageData[y*WIDTH*HEIGHT + l*WIDTH + (WIDTH-x)] >= iThreshold){
                    m_pDepthBuffer[y*WIDTH + x] = l;
                    break;
                }
                else {
                    m_pDepthBuffer[y*WIDTH + x] = 0;
                }
            }
        }
    }
    return 0;
}

/**
 * @brief CTDataset::renderDepthBuffer: Renders the depth buffer to a lighting model, using the angle of the surface to the lightsource
 * @param shadedBuffer a 2D lighting model displaying the topography of a 3D object from a certain direction
 * @return 0
 */
int CTDataset::renderDepthBuffer(short* shadedBuffer){
    float incidence_angle;
    float T_x;
    float T_y;
    for (int y=1; y < HEIGHT-1; ++y){
        for (int x=1; x < WIDTH-1; ++x){
            T_x = m_pDepthBuffer[y*WIDTH + (x-1)] - m_pDepthBuffer[y*WIDTH + (x+1)];
            T_y = m_pDepthBuffer[(y-1)*WIDTH + x] - m_pDepthBuffer[(y+1)*WIDTH + x];
            incidence_angle = (2*2)/(sqrt(pow(2*T_x, 2) + pow(2*T_y, 2) + pow(2*2, 2) ));
            shadedBuffer[y*WIDTH + x] = 255 * incidence_angle;
        }
    }
    return 0;
}

/**
 * @brief CTDataset::regionGrowing performs region growing
 * @param seed the voxel from which to start region growing
 * @param threshold the threshold at which region growing stops
 * @param iRegion a list of all voxels that are found to be in the created region
 * @return 0 - if successful, 1 - if seed is invalid, 2 - if seed is below threshold
 */
int CTDataset::regionGrowing(Voxel seed, int threshold, std::vector <Voxel>& iRegion){
    std::vector <Voxel> Searchlist;
    Voxel voxel;

    // 1: seed out of bounds
    if (seed.x < 0 || seed.y < 0 || seed.z < 0){
        return 1; //seed invalid
    }

    // Set seed on searchlist if above threshold
    if (m_pImageData[seed.z*WIDTH*HEIGHT + seed.y*WIDTH + seed.x] >= threshold){
        Searchlist.push_back(seed);
    }
    else {
        return 2; // 2: seed below threshold
    }

    while (!Searchlist.empty()){
        // Read last voxel in searchlist and delete it from list
        voxel = Searchlist.back();
        Searchlist.pop_back();
        iRegion.push_back(voxel);

        visited_voxel[voxel.z*WIDTH*HEIGHT + voxel.y*WIDTH + voxel.x] = true;

        // Only look at voxels in scope of frame
        if (0 < voxel.x & voxel.x < WIDTH & 0 < voxel.y & voxel.y < HEIGHT & 0 < voxel.z & voxel.z < LAYERS-1){
            m_pRegionData[voxel.z*WIDTH*HEIGHT + voxel.y*WIDTH + voxel.x] = m_pImageData[voxel.z*WIDTH*HEIGHT + voxel.y*WIDTH + voxel.x];

            // Add neighbors to searchlist if not visited and above threshold
            if (!visited_voxel[voxel.z*WIDTH*HEIGHT + voxel.y*WIDTH + (voxel.x+1)] && m_pImageData[voxel.z*WIDTH*HEIGHT + voxel.y*WIDTH + (voxel.x+1)] >= threshold){ Searchlist.push_back({voxel.x+1, voxel.y, voxel.z}); }
            if (!visited_voxel[voxel.z*WIDTH*HEIGHT + voxel.y*WIDTH + (voxel.x-1)] && m_pImageData[voxel.z*WIDTH*HEIGHT + voxel.y*WIDTH + (voxel.x-1)] >= threshold){ Searchlist.push_back({voxel.x-1, voxel.y, voxel.z}); }
            if (!visited_voxel[voxel.z*WIDTH*HEIGHT + (voxel.y+1)*WIDTH + voxel.x] && m_pImageData[voxel.z*WIDTH*HEIGHT + (voxel.y+1)*WIDTH + voxel.x] >= threshold){ Searchlist.push_back({voxel.x, voxel.y+1, voxel.z}); }
            if (!visited_voxel[voxel.z*WIDTH*HEIGHT + (voxel.y-1)*WIDTH + voxel.x] && m_pImageData[voxel.z*WIDTH*HEIGHT + (voxel.y-1)*WIDTH + voxel.x] >= threshold){ Searchlist.push_back({voxel.x, voxel.y-1, voxel.z}); }
            if (!visited_voxel[(voxel.z+1)*WIDTH*HEIGHT + voxel.y*WIDTH + voxel.x] && m_pImageData[(voxel.z+1)*WIDTH*HEIGHT + voxel.y*WIDTH + voxel.x] >= threshold){ Searchlist.push_back({voxel.x, voxel.y, voxel.z+1}); }
            if (!visited_voxel[(voxel.z-1)*WIDTH*HEIGHT + voxel.y*WIDTH + voxel.x] && m_pImageData[(voxel.z-1)*WIDTH*HEIGHT + voxel.y*WIDTH + voxel.x] >= threshold){ Searchlist.push_back({voxel.x, voxel.y, voxel.z-1}); }
        }
    }
    return 0;
}

/**
 * @brief CTDataset::getRegistrationMarkers determines all registration markers and saves them to m_pRegionData
 * @param threshold the threshold chosen to single out the markers
 */
void CTDataset::getRegistrationMarkers(int threshold){
    Voxel seed;
    std::vector<std::vector<Voxel>> regions;

    // Clean visited_voxel array
    for (int i=0; i<WIDTH*HEIGHT*LAYERS; i++) {
        visited_voxel[i] = false;
    }

    // get regions
    QElapsedTimer timer;
    timer.start();
    for (int z=0; z<LAYERS; z+=2){
        for (int y=0; y<HEIGHT; y+=2){
            for (int x=0; x<WIDTH; x+=2){
                std::vector <Voxel> region;
                seed.x = x;
                seed.y = y;
                seed.z = z;
                regionGrowing(seed, threshold, region);
                if (250 < region.size() && region.size() < 1000) {
                    // version as specified by the instructions (120<x<200 and 230<x<400 but rotated)
                    /*if ((280 >= getMaxX(region) && getMinX(region) >= 200) || (170 >= getMaxX(region) && getMinX(region) >= 0)){
                        regions.push_back(region);
                    }*/
                    // version that works better
                    if (getWidth(region) > 5 && getWidth(region) < 20 && getWidth(region) > 5 && getWidth(region) < 20){
                        regions.push_back(region);
                        getCentroid(region);
                    }
                }
            }
        }
    }

    // Empty the region data
    for (int i=0; i<LAYERS*HEIGHT*WIDTH; i++) {
        m_pRegionData[i] = -1024;
    }
    // Iterate over regions and write them to region data
    int index;
    for (unsigned long int i = 0; i < regions.size(); i++) {
        for (unsigned long int j = 0; j < regions[i].size(); j++) {
            index = regions[i][j].z*HEIGHT*WIDTH + regions[i][j].y*WIDTH + regions[i][j].x;
            m_pRegionData[index] = m_pImageData[index];
        }
    }
}

/**
 * @brief CTDataset::getCentroid calculates the average of all vectors in a region and saves it to markerCentroids
 * @param region the region of which the centroid is computed
 */
void CTDataset::getCentroid(std::vector<Voxel> region){
    Voxel voxel;
    Voxel centroid = {0, 0, 0};

    int i = 0;
    while (!region.empty()){
        voxel = region.back();
        region.pop_back();
        centroid.x += voxel.x;
        centroid.y += voxel.y;
        centroid.z += voxel.z;
        i++;
    }
    centroid.x /= i;
    centroid.y /= i;
    centroid.z /= i;

    markerCentroids.push_back(centroid);
}

/**
 * @brief CTDataset::getMaxX
 * @param region
 * @return value of maximal x-coordinate in the region
 */
int CTDataset::getMaxX(std::vector <Voxel> region){
    int max = 0;
    Voxel voxel;
    while (!region.empty()){
        voxel = region.back();
        region.pop_back();
        if (max < voxel.x)
            max = voxel.x;
    }
    return max;
}

/**
 * @brief CTDataset::getMinX
 * @param region
 * @return value of minimal x-coordinate in the region
 */
int CTDataset::getMinX(std::vector <Voxel> region){
    int min = WIDTH;
    Voxel voxel;
    while (!region.empty()){
        voxel = region.back();
        region.pop_back();
        if (min > voxel.x)
            min = voxel.x;
    }
    return min;
}

/**
 * @brief CTDataset::getWIDTH
 * @param region
 * @return WIDTH of the region
 */
int CTDataset::getWidth(std::vector <Voxel> region){
    int maxX = 0;
    int minX = WIDTH;
    Voxel voxel;
    while (!region.empty()){
        voxel = region.back();
        region.pop_back();
        if (maxX < voxel.x)
            maxX = voxel.x;
        if (voxel.x < minX)
            minX = voxel.x;
    }
    return maxX-minX;
}

/**
 * @brief CTDataset::getHEIGHT
 * @param region
 * @return HEIGHT of the region
 */
int CTDataset::getHeight(std::vector<Voxel> region){
    int maxY = 0;
    int minY = HEIGHT;
    Voxel voxel;
    while (!region.empty()){
        voxel = region.back();
        region.pop_back();
        if (maxY < voxel.y)
            maxY = voxel.y;
        if (voxel.y < minY)
            minY = voxel.y;
    }
    return maxY-minY;
}

/**
 * @brief CTDataset::registerMarkers Enters source points (found centroids) into sourcePoints list, performs calculate and stores resulting transformation matrix
 */
void CTDataset::registerMarkers(){
    IcpAlgo icp;
    Voxel voxel;
    for (unsigned long int i=0; i<markerCentroids.size(); i++){
        voxel = markerCentroids[i];

        //revert rotations back to original
        Eigen::Vector3d a((WIDTH-voxel.x-2)*0.3625, (HEIGHT-voxel.y)*0.325, voxel.z*0.35);
        icp.sourcePoints.push_back(a);
    }

    icp.calculate();
    resultMatrixInverse = icp.resultMatrix.matrix().inverse();
    resultMatrixRotation = icp.resultMatrix.rotation().inverse();
}

/**
 * @brief CTDataset::reconstructLayer
 * @param pos center of image
 * @param axis
 * @param xdir Vorzugsachse (soll entweder (1,0,0) oder (0,0,1) sein)
 */
void CTDataset::reconstructLayer(Voxel posVoxel, Voxel axisVoxel, Voxel xdirVoxel){
    // Convert Voxels to Eigen
    Eigen::Vector3d pos(posVoxel.x, posVoxel.y, posVoxel.z);
    Eigen::Vector3d axis(axisVoxel.x, axisVoxel.y, axisVoxel.z);
    Eigen::Vector3d xdir(xdirVoxel.x, xdirVoxel.y, xdirVoxel.z);
    axis.normalize();

    Eigen::Vector3d tmpDir = axis.cross(xdir);
    Eigen::Vector3d xImDir = tmpDir.cross(axis);
    Eigen::Vector3d yImDir = axis;
    xImDir.normalize();

    int h = HEIGHT/2;
    int w = WIDTH/2;
    for (int y = -h; y < h; y++){
        for (int x = -w; x < w; x++){
            Eigen::Vector3d pos3d = pos + x*xImDir + y*yImDir;
            pos3d.x() = (int)std::round(pos3d.x());
            pos3d.y() = (int)std::round(pos3d.y());
            pos3d.z() = (int)std::round(pos3d.z());
            if (0 <= pos3d.x() && pos3d.x() < WIDTH && 0 <= pos3d.y() && pos3d.y() < HEIGHT && 0 <= pos3d.z() && pos3d.z() < LAYERS){
                crosssectionImageData[(y+h)*WIDTH + (x+w)] =
                        m_pImageData[(int)pos3d.z()*HEIGHT*WIDTH + (HEIGHT-(int)pos3d.y())*WIDTH + (WIDTH-(int)pos3d.x())];
            }
            else {
                crosssectionImageData[(y+h)*WIDTH + (x+w)] = -1024;
            }
        }
    }
}

/**
 * @brief CTDataset::reconstructLayer_world
 * @param worldPos tip of instrument in world coordinates
 * @param worldAxis vector of instrument handle in world coordinates
 * @param xdir Vorzugsachse
 */
void CTDataset::reconstructLayer_world(Eigen::Vector3d worldPos, Eigen::Vector3d worldAxis, Voxel xdir){
    IcpAlgo icp;
    Eigen::Vector3d voxellengths3d(0.3625, 0.325, 0.35);
    Eigen::Vector4d voxellengths4d(0.3625, 0.325, 0.35, 1);

    Eigen::Vector4d tmp(worldPos.x(), worldPos.y(), worldPos.z(), 1);
    tmp = resultMatrixInverse*tmp;
    tmp = tmp.cwiseQuotient(voxellengths4d);
    Eigen::Vector3d pos(tmp.x(), tmp.y(), tmp.z());

    Eigen::Vector3d axis = resultMatrixRotation*worldAxis;
    axis = axis.cwiseQuotient(voxellengths3d).normalized();

    Eigen::Vector3d dir(xdir.x, xdir.y, xdir.z);

    Eigen::Vector3d tmpDir = axis.cross(dir);
    Eigen::Vector3d xImDir = tmpDir.cross(axis);
    Eigen::Vector3d yImDir = axis;
    xImDir.normalize();

    int h = HEIGHT/2;
    int w = WIDTH/2;
    for (int y = -h; y < h; y++){
        for (int x = -w; x < w; x++){
            Eigen::Vector3d pos3d = pos + x*xImDir + y*yImDir;
            //qDebug() << pos3d.x() << ", " << pos3d.y() << ", " << pos3d.z();
            pos3d.x() = (int)std::round(pos3d.x());
            pos3d.y() = (int)std::round(pos3d.y());
            pos3d.z() = (int)std::round(pos3d.z());
            if (0 <= pos3d.x() && pos3d.x() < WIDTH && 0 <= pos3d.y() && pos3d.y() < HEIGHT && 0 <= pos3d.z() && pos3d.z() < LAYERS){
                crosssectionImageData[(y+h)*WIDTH + (x+w)] =
                        m_pImageData[(int)pos3d.z()*HEIGHT*WIDTH + (HEIGHT-(int)pos3d.y())*WIDTH + (WIDTH-(int)pos3d.x())];
            }
            else {
                crosssectionImageData[(y+h)*WIDTH + (x+w)] = -1024;
            }
        }
    }
}
