#include "widget.h"
#include "ui_widget.h"
#include "ctdataset.h"
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QElapsedTimer>
#include <QDebug>
#include <QMouseEvent>
#include <cmath>
#include "Eigen/Core"
#include "Eigen/Dense"


Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    // Buttons
    connect(ui->pushButton_loadImage, SIGNAL(clicked()), this, SLOT(loadImage()));
    connect(ui->pushButton_render3D, SIGNAL(clicked()), this, SLOT(Render3D()));
    connect(ui->pushButton_regionGrowing, SIGNAL(clicked()), this, SLOT(startRegionGrowing()));
    connect(ui->pushButton_markers, SIGNAL(clicked()), this, SLOT(getMarkers()));
    connect(ui->pushButton_updateCrosssections, SIGNAL(clicked()), this, SLOT(performWorldLayerReconstruction()));

    // Sliders
    connect(ui->horizontalSlider_startValue, SIGNAL(valueChanged(int)), this, SLOT(updatedWindowingStart(int)));
    connect(ui->horizontalSlider_windowWidth, SIGNAL(valueChanged(int)), this, SLOT(updatedWindowingWidth(int)));
    connect(ui->horizontalSlider_layerNumber, SIGNAL(valueChanged(int)), this, SLOT(updatedLayerNumber(int)));
    connect(ui->horizontalSlider_thresholdValue, SIGNAL(valueChanged(int)), this, SLOT(updatedThresholdValue(int)));

    // Spin boxes
    connect(ui->spinBox_LocalX, SIGNAL(valueChanged(int)), this, SLOT(performLayerReconstruction()));
    connect(ui->spinBox_LocalY, SIGNAL(valueChanged(int)), this, SLOT(performLayerReconstruction()));
    connect(ui->spinBox_LocalZ, SIGNAL(valueChanged(int)), this, SLOT(performLayerReconstruction()));
    connect(ui->spinBox_WorldX, SIGNAL(valueChanged(int)), this, SLOT(performWorldLayerReconstruction()));
    connect(ui->spinBox_WorldY, SIGNAL(valueChanged(int)), this, SLOT(performWorldLayerReconstruction()));
    connect(ui->spinBox_WorldZ, SIGNAL(valueChanged(int)), this, SLOT(performWorldLayerReconstruction()));

    imageLoaded = false;
    depthBufferCreated = false;
    validVoxelSelected = false;
    markersLocated = false;
}



Widget::~Widget()
{
    delete ui;
}

void Widget::loadImage()
{
    // open File Dialog to select dataset
    QString imagePath = QFileDialog::getOpenFileName(this, "Open Image", "./", "Raw Image Files (*.raw)");

    // try to load dataset
    int iErrorCode = dataset.load(imagePath);
    if (iErrorCode == 0){
        imageLoaded = true;
        updateSliceView();
        getMarkers();
        performWorldLayerReconstruction();
        Render3D();
    }
    else {
        if (iErrorCode == 1){
            QMessageBox::critical(this, "Warning", "File could not be opened.");
        } else if (iErrorCode == 2){
            QMessageBox::critical(this, "Warning", "There is an issue with your image. Please make sure it has the correct format.");
        } else if (iErrorCode == 3){
            QMessageBox::critical(this, "Warning", "Unexpected count of read bytes.");
        }

    }
}

void Widget::Render3D(){
    if (imageLoaded){
        QImage image(width, height, QImage::Format_RGB32);
        image.fill(qRgb(255, 255, 255));

        short shadedBuffer[width*height];

        int threshold = ui->horizontalSlider_thresholdValue->value();

        // Calculate depthBuffer and set depthBufferCreated to true if successful
        if (dataset.calculateDepthBuffer(threshold, dataset.data()) == 0){
            depthBufferCreated = true;
        }
        else {
            QMessageBox::critical(this, "Warning", "Depth buffer couldn't be calculated.");
        }

        dataset.renderDepthBuffer(shadedBuffer);

        //draw shadedBuffer to image
        int color;
        for (int y=0; y < height; ++y){
            for(int x=0; x < width; ++x){
                color = shadedBuffer[y*width + x];
                image.setPixel(x, y, qRgb(color, color, color));
            }
        }

        ui->label_image3D->setPixmap(QPixmap::fromImage(image));
    }
    else {
        QMessageBox::critical(this, "Warning", "No image loaded yet.");
    }

}

//--------------------------------------------------------------
//  Slider updates
//--------------------------------------------------------------

void Widget::updatedWindowingStart(int value){
    ui->label_startValue->setText("Start Value: " + QString::number(value));
    updateSliceView();
}

void Widget::updatedWindowingWidth(int value){
    ui->label_windowWidth->setText("Window Width: " + QString::number(value));
    updateSliceView();
}

void Widget::updatedLayerNumber(int value){
    ui->label_layerNumber->setText("Layer: " + QString::number(value));
    updateSliceView();
}

void Widget::updatedThresholdValue(int value){
    ui->label_thresholdValue->setText("Threshold: " + QString::number(value));
    updateSliceView();
}

//--------------------------------------------------------------

void Widget::updateSliceView(){
    //Initialize performance check
    QElapsedTimer timer;
    timer.start();

    //Variable vom Typ QImage der Größe 512*512 erzeugen und schwarz füllen
    QImage image(width, height, QImage::Format_RGB32);
    image.fill(qRgb(0, 0, 0));

    int errorCode = 0;
    int iGrayvalue = 0;

    int layerconstant = height*width * ui->horizontalSlider_layerNumber->value();
    int startValueValue = ui->horizontalSlider_startValue->value();
    int windowWidthValue = ui->horizontalSlider_windowWidth->value();
    int thresholdValueValue = ui->horizontalSlider_thresholdValue->value();    

    //In einer Doppelschleife über y und x jeweils den zugehörigen index des Speichers berechnen
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            //read Grayvalue at index from m_pImageData/dataset.data()
            errorCode = CTDataset::windowing(dataset.data()[layerconstant + y*width + x], startValueValue, windowWidthValue, iGrayvalue);
            // if HU Value exceeds segmenting threshold
            if (dataset.data()[layerconstant + y*width + x] >= thresholdValueValue){
                image.setPixel(x, y, qRgb(255, 0, 0));
            }
            else {
                //set pixel at (x,y) to Grayvalue
                image.setPixel(x, y, qRgb(iGrayvalue, iGrayvalue, iGrayvalue));
            }
        }
    }

    //Abschließend das image als Pixmap in das Label setzen
    ui->label_image->setPixmap(QPixmap::fromImage(image));

    if (markersLocated && ui->checkBox_autoUpdateCrosssections->isChecked()){
        performWorldLayerReconstruction();
    }

    //qDebug() << timer.nsecsElapsed();
}

void Widget::mousePressEvent(QMouseEvent *event){
    QPoint globalPos = event->pos();
    QPoint imagePos = (ui->label_image->mapFromParent(globalPos));
    QPoint image3DPos = (ui->label_image3D->mapFromParent(globalPos));

    // if clicked in image
    if (ui->label_image->rect().contains(imagePos)){
        ui->label_X->setText("X: " + QString::number(width - imagePos.x()));
        ui->label_X_real->setText("X: " + QString::number((width - imagePos.x())*0.3625) + "mm"); //real
        voxel.x = width - imagePos.x();
        ui->label_Y->setText("Y: " + QString::number(height - imagePos.y()));
        ui->label_Y_real->setText("Y: " + QString::number((height - imagePos.y())*0.325) + "mm"); //real
        voxel.y = height - imagePos.y();
        if (depthBufferCreated){
            ui->label_Z->setText("Z: " + QString::number(ui->horizontalSlider_layerNumber->value()));
            ui->label_Z_real->setText("Z: " + QString::number(ui->horizontalSlider_layerNumber->value()*0.35) + "mm");
            voxel.z = ui->horizontalSlider_layerNumber->value();
            validVoxelSelected = true;
        }
        else {
            ui->label_Z->setText("Z: -");
        }
    }

    // if clicked in image3D
    if (ui->label_image3D->rect().contains(image3DPos)){
        ui->label_X->setText("X: " + QString::number(image3DPos.x()));
        ui->label_X_real->setText("X: " + QString::number(image3DPos.x()*0.3625) + "mm");
        voxel.x = width - image3DPos.x();
        ui->label_Z->setText("Z: " + QString::number(image3DPos.y()));
        ui->label_Z_real->setText("Z: " + QString::number(image3DPos.y()*0.35) + "mm");
        voxel.z = image3DPos.y();
        if (depthBufferCreated){
            ui->label_Y->setText("Y: " + QString::number(dataset.depthbuffer()[image3DPos.y()*width + image3DPos.x()]));
            ui->label_Y_real->setText("Y: " + QString::number((dataset.depthbuffer()[image3DPos.y()*width + image3DPos.x()])*0.325) + "mm");
            voxel.y = dataset.depthbuffer()[image3DPos.y()*width + image3DPos.x()];
            validVoxelSelected = true;
        }
        else {
            ui->label_Y->setText("Y: -");
        }
    }

}

void Widget::startRegionGrowing(){
    QImage image(width, height, QImage::Format_RGB32);
    image.fill(qRgb(255, 255, 255));
    short shadedBuffer[width*height];

    std::vector <Voxel> region;
    int errorCode;

    if (!validVoxelSelected){
        errorCode = 3;
        QMessageBox::critical(this, "Error", "No valid seed selected");
    }
    else{
        // Clear region storage and visited_voxel list
        for (int i=0; i < width*height*layers; i++){
            dataset.region()[i] = 0;
            dataset.visited_voxel[i] = false;
        }

        // Perform region growing
        int threshold = ui->horizontalSlider_thresholdValue->value();
        if (voxel.x >= 0 && voxel.y >= 0 && voxel.z >= 0){
            errorCode = dataset.regionGrowing(voxel, threshold, region);
        } else {
            errorCode = 1; // Voxel invalid
        }

        if (errorCode == 0){ // Everything OK
            dataset.calculateDepthBuffer(threshold, dataset.region());

            dataset.renderDepthBuffer(shadedBuffer);

            //draw depthbuffer to image
            int color;
            for (int y=0; y < height; ++y){
                for(int x=0; x < width; ++x){
                    color = shadedBuffer[y*width + x];
                    image.setPixel(x,y,qRgb(color, color, color));
                }
            }

            ui->label_image3D->setPixmap(QPixmap::fromImage(image));
        }
        else if (errorCode == 1) { QMessageBox::critical(this, "Error", "Invalid seed"); }
        else if (errorCode == 2) { QMessageBox::critical(this, "Error", "Seed below threshold"); }
    }
}

void Widget::getMarkers(){
    if (imageLoaded){
        QImage image(width, height, QImage::Format_RGB32);
        image.fill(qRgb(255, 255, 255));

        Voxel centroid;
        short shadedBuffer[width*height];

        dataset.getRegistrationMarkers(1500);

        // get depth map of marker regions
        dataset.calculateDepthBuffer(1500, dataset.region());
        dataset.renderDepthBuffer(shadedBuffer);

        dataset.registerMarkers();

        // draw shadedBuffer to image
        int color;
        for (int y=0; y < height; ++y){
            for(int x=0; x < width; ++x){
                color = shadedBuffer[y*width + x];
                image.setPixel(x, y, qRgb(color, color, color));
            }
        }

        // draw marker centroids to image
        while (!dataset.markerCentroids.empty()){
            centroid = dataset.markerCentroids.back();
            dataset.markerCentroids.pop_back();

            // draw cross on centroid position
            for (int i=-2; i<=2; i++){
                for (int j=-2; j<=2; j++){
                    image.setPixel((width-centroid.x)+i, centroid.z+j, qRgb(255, 0, 0));
                }
            }
        }

        ui->label_image3D->setPixmap(QPixmap::fromImage(image));
        markersLocated = true;
    }
    else {
        QMessageBox::critical(this, "Warning", "Can't calculate registration markers.");
    }
}

void Widget::performLayerReconstruction(){
    Voxel pos = {108, 194, 129};
    Voxel axis = {1, 5, 1};
    Voxel xdir = {1, 0, 0};

    int x = ui->spinBox_LocalX->value();
    int y = ui->spinBox_LocalY->value();
    int z = ui->spinBox_LocalZ->value();
    pos = {x, y, z};

    dataset.reconstructLayer(pos, axis, xdir);

    // loop over crosssectionImageData to create image_Xdir
    int iGrayvalue = 0;
    int startValueValue = ui->horizontalSlider_startValue->value();
    int windowWidthValue = ui->horizontalSlider_windowWidth->value();
    QImage image(width, height, QImage::Format_RGB32);
    image.fill(qRgb(255, 255, 255));
    for (int y=0; y < height; ++y){
        for(int x=0; x < width; ++x){
            CTDataset::windowing(dataset.crosssectionImageData[y*width + x], startValueValue, windowWidthValue, iGrayvalue);
            image.setPixel(x, y, qRgb(iGrayvalue, iGrayvalue, iGrayvalue));
        }
    }
    drawInstrumentOverlay(image);
    ui->label_image_Xdir->setPixmap(QPixmap::fromImage(image));

    // loop over crosssectionImageData to create image_Zdir
    xdir = {0, 0, 1};
    dataset.reconstructLayer(pos, axis, xdir);
    image.fill(qRgb(255, 255, 255));
    for (int y=0; y < height; ++y){
        for(int x=0; x < width; ++x){
            CTDataset::windowing(dataset.crosssectionImageData[y*width + x], startValueValue, windowWidthValue, iGrayvalue);
            image.setPixel(x, y, qRgb(iGrayvalue, iGrayvalue, iGrayvalue));
        }
    }
    drawInstrumentOverlay(image);
    ui->label_image_Zdir->setPixmap(QPixmap::fromImage(image));
}

void Widget::performWorldLayerReconstruction(){
    if (markersLocated){
        int iGrayvalue = 0;
        int startValueValue = ui->horizontalSlider_startValue->value();
        int windowWidthValue = ui->horizontalSlider_windowWidth->value();
        QImage image(width, height, QImage::Format_RGB32);
        Eigen::Vector3d worldPos = {-15, -65, -57};
        Eigen::Vector3d worldAxis = {0.688, -0.688, 0.23};
        Voxel xdir = {1, 0, 0};

        int x = ui->spinBox_WorldX->value();
        int y = ui->spinBox_WorldY->value();
        int z = ui->spinBox_WorldZ->value();
        worldPos = {x, y, z};


        // loop over crosssectionImageData to create image_Xdir
        dataset.reconstructLayer_world(worldPos, worldAxis, xdir);
        image.fill(qRgb(255, 255, 255));
        for (int y=0; y < height; ++y){
            for(int x=0; x < width; ++x){
                CTDataset::windowing(dataset.crosssectionImageData[y*width + x], startValueValue, windowWidthValue, iGrayvalue);
                image.setPixel(x, y, qRgb(iGrayvalue, iGrayvalue, iGrayvalue));
            }
        }
        drawInstrumentOverlay(image);
        ui->label_image_Xdir->setPixmap(QPixmap::fromImage(image));

        // loop over crosssectionImageData to create image_Zdir
        xdir = {0, 0, 1};
        dataset.reconstructLayer_world(worldPos, worldAxis, xdir);
        image.fill(qRgb(255, 255, 255));
        for (int y=0; y < height; ++y){
            for(int x=0; x < width; ++x){
                CTDataset::windowing(dataset.crosssectionImageData[y*width + x], startValueValue, windowWidthValue, iGrayvalue);
                image.setPixel(x, y, qRgb(iGrayvalue, iGrayvalue, iGrayvalue));
            }
        }
        drawInstrumentOverlay(image);
        ui->label_image_Zdir->setPixmap(QPixmap::fromImage(image));
    }
    else {
         QMessageBox::critical(this, "Warning", "Markers not registered yet.");
    }
}

void Widget::drawInstrumentOverlay(QImage &image){
    for (int y=0; y< height/2; y++){
        for (int x=(width/2)-2; x<=(width/2)+2; x++){
            image.setPixel(x, y, qRgb(255, 0, 0));
        }
    }
    for (int y=height/2; y< height; y++){
        for (int x=(width/2)-1; x<=(width/2)+1; x++){
            if ((y/8)%2 == 0){
                image.setPixel(x, y, qRgb(255, 0, 0));
            }

        }
    }

}
