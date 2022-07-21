#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "ctdataset.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private:
    Ui::Widget *ui;

    void updateSliceView();

    void drawInstrumentOverlay(QImage &image);

    CTDataset dataset;
    Voxel voxel;

    int width = 400;
    int height = 400;
    int layers = 400;

    bool imageLoaded;
    bool depthBufferCreated;
    bool validVoxelSelected;
    bool markersLocated;

private slots:
    void loadImage();

    void updatedWindowingStart(int value);
    void updatedWindowingWidth(int value);
    void updatedLayerNumber(int value);
    void updatedThresholdValue(int value);

    void Render3D();
    void startRegionGrowing();

    void mousePressEvent(QMouseEvent *event);

    void getMarkers();
    void performLayerReconstruction();
    void performWorldLayerReconstruction();
};
#endif // WIDGET_H
