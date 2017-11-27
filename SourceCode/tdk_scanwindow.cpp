#include "tdk_scanwindow.h"

TDK_ScanWindow::TDK_ScanWindow(QMainWindow *parent) : QMainWindow(parent),
    mv_CentralWidget                        (new QWidget(this))                                 ,
    mv_StatusBar                            (new QStatusBar)                                    ,
    mv_CentralGridLayout                    (new QGridLayout)                                   ,
    mv_SensorController                     (new TDK_SensorController)                          ,
    mv_SensorComboBox                       (new QComboBox)                                     ,
    mv_Sensor                               (nullptr)                                           ,
    mv_PointCloudStreamQVTKWidget           (new QVTKWidget)                                    ,
    mv_FlagRealTimeScan                     (false)                                             ,
    mv_FlagScanning                         (false)                                             ,
    mv_XMinimumSpinBox                      (new QDoubleSpinBox)                                ,
    mv_XMaximumSpinBox                      (new QDoubleSpinBox)                                ,
    mv_YMinimumSpinBox                      (new QDoubleSpinBox)                                ,
    mv_YMaximumSpinBox                      (new QDoubleSpinBox)                                ,
    mv_ZMinimumSpinBox                      (new QDoubleSpinBox)                                ,
    mv_ZMaximumSpinBox                      (new QDoubleSpinBox)                                ,
    mv_InclinationSpinBox                   (new QDoubleSpinBox)                                ,
    mv_FilterBoxCheckBox                    (new QCheckBox)                                     ,
    mv_RegistrationCheckBox                 (new QCheckBox)                                     ,
    mv_CapturePointCloudPushButton          (new QPushButton(QString("CAPTURE POINT CLOUD")))   ,
    mv_StartScanPushButton                  (new QPushButton(QString("START SCAN")))            ,
    mv_StopScanPushButton                   (new QPushButton(QString("STOP SCAN")))             ,
    mv_PlatformParametersYesRadioButton     (new QRadioButton(QString("Yes")))                  ,
    mv_PlatformParametersNoRadioButton      (new QRadioButton(QString("No")))                   ,
    mv_IncrementalRotationAngleSpinBox      (new QDoubleSpinBox)                                ,
    mv_NumberOfRotationsSpinBox             (new QDoubleSpinBox)                                ,
    mv_FlagTurnTableParametersEnabled       (false)                                             ,
    mv_FlagPointCloudExists                 (false)                                             ,
    mv_NumberOfPointCloudsCaptured          (0)                                                 ,
    mv_NumberOfPointCloudsCapturedLabel     (new QLabel("0"))                                   ,
    mv_ScanRegistration                     (new TDK_ScanRegistration)                          ,
    mv_SerialPortNameLineEdit               (new QLineEdit)                                     ,
    mv_SerialPortBaudRateComboBox           (new QComboBox)                                     ,
    mv_Turntable                            (new TDK_Turntable)
{

    this->setStatusBar(mv_StatusBar);

    connect(mv_SensorComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(mf_SlotUpdateWindow(int)));
    connect(mv_XMinimumSpinBox, SIGNAL(valueChanged(double)), this, SLOT(mf_SlotUpdateBoundingBox()));
    connect(mv_XMaximumSpinBox, SIGNAL(valueChanged(double)), this, SLOT(mf_SlotUpdateBoundingBox()));
    connect(mv_YMinimumSpinBox, SIGNAL(valueChanged(double)), this, SLOT(mf_SlotUpdateBoundingBox()));
    connect(mv_YMaximumSpinBox, SIGNAL(valueChanged(double)), this, SLOT(mf_SlotUpdateBoundingBox()));
    connect(mv_ZMinimumSpinBox, SIGNAL(valueChanged(double)), this, SLOT(mf_SlotUpdateBoundingBox()));
    connect(mv_ZMaximumSpinBox, SIGNAL(valueChanged(double)), this, SLOT(mf_SlotUpdateBoundingBox()));
    connect(mv_InclinationSpinBox, SIGNAL(valueChanged(double)), this, SLOT(mf_SlotUpdateBoundingBox()));
    connect(mv_RegistrationCheckBox, SIGNAL(clicked(bool)), this, SLOT(mf_SlotPointCloudRegistration(bool)));
    connect(mv_FilterBoxCheckBox, SIGNAL(clicked(bool)), this, SLOT(mf_SlotActivateFiltering(bool)));
    connect(mv_StartScanPushButton, SIGNAL(clicked(bool)), this, SLOT(mf_SlotStartScan()));
    connect(mv_StopScanPushButton, SIGNAL(clicked(bool)), this, SLOT(mf_SlotStopScan()));
    connect(mv_CapturePointCloudPushButton, SIGNAL(clicked(bool)), this, SLOT(mf_SlotCapturePointCloudButtonClick()));
    connect(mv_PlatformParametersYesRadioButton, SIGNAL(toggled(bool)), this, SLOT(mf_SlotHandlePlatformParameters(bool)));
    connect(this, SIGNAL(mf_SignalNumberOfPointCloudUpdated(int)), mv_NumberOfPointCloudsCapturedLabel, SLOT(setNum(int)));
    connect(mv_Turntable, SIGNAL(mf_SignalStepAngleRotated(int)), this, SLOT(mf_SlotCapturePointCloud(int)));
    connect(mv_Turntable, SIGNAL(mf_SignalRotationsDone()), this, SLOT(mf_SlotStopScan()));

    connect(this, SIGNAL(mf_SignalStatusChanged(QString,QColor)), this, SLOT(mf_SlotUpdateStatusBar(QString,QColor)));

}

TDK_ScanWindow::~TDK_ScanWindow()
{

}

void TDK_ScanWindow::mf_setupUI()
{
    vtkObject::GlobalWarningDisplayOff();
    this->setWindowFlags(Qt::Dialog);
    //resize(300, 400);
    setCentralWidget(mv_CentralWidget);

    mf_SetupPointCloudStreamWidget();
    mf_SetupSensorWidget();
    mf_SetupPlatformParametersWidget();

    mv_CentralGridLayout->setColumnMinimumWidth(0, 300);
    mv_CentralGridLayout->setColumnMinimumWidth(1, 300);

    mv_CentralGridLayout->setRowMinimumHeight(0, 300);
    mv_CentralGridLayout->setRowMinimumHeight(1, 300);

    mv_CentralGridLayout->setColumnStretch(1, 1);

    mv_CentralWidget->setLayout(mv_CentralGridLayout);
}

void TDK_ScanWindow::mf_SetupPointCloudStreamWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Point Cloud Visualizer"));
    dockWidget->setWidget(mv_PointCloudStreamQVTKWidget);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 0, 1, 2, 1);

    mv_PointCloudStreamVisualizer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    mv_PointCloudStreamVisualizer->setBackgroundColor (0.1, 0.1, 0.1);
    mv_PointCloudStreamVisualizer->setCameraPosition( 0.0, 0.0, -5, 0.0, 0.0, 0.0 );

    mv_PointCloudStreamVisualizer->addCoordinateSystem(0.5);

    mv_PointCloudStreamVisualizer->addCube(mv_XMinimumSpinBox->value(), mv_XMaximumSpinBox->value(), mv_YMinimumSpinBox->value(), mv_YMaximumSpinBox->value(), mv_ZMinimumSpinBox->value(), mv_ZMaximumSpinBox->value());
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.33, "cube");
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.0, 0.0, "cube");
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "cube");

    //Add turntableAxisOfRotation
    float x_coord = mv_XMinimumSpinBox->value() + (mv_XMaximumSpinBox->value() - mv_XMinimumSpinBox->value())/2;
    float z_coord = mv_ZMinimumSpinBox->value() + (mv_ZMaximumSpinBox->value() - mv_ZMinimumSpinBox->value())/2;

    float inclinationDegrees = mv_InclinationSpinBox->value();
    float top_z_coord = z_coord - (0 - mv_YMinimumSpinBox->value())*tan(inclinationDegrees*M_PI/180.0);
    float bottom_z_coord = z_coord;

    pcl::PointXYZ top(x_coord, mv_YMaximumSpinBox->value(), top_z_coord);
    pcl::PointXYZ bottom(x_coord, mv_YMinimumSpinBox->value(), bottom_z_coord);

    mv_PointCloudStreamVisualizer->addLine<pcl::PointXYZ>(bottom, top, 0.0, 0.9, 0.0, "rot_axis");
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "rot_axis");
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "rot_axis");

    mv_PointCloudStreamQVTKWidget->SetRenderWindow ( mv_PointCloudStreamVisualizer->getRenderWindow () );
    mv_PointCloudStreamVisualizer->setupInteractor ( mv_PointCloudStreamQVTKWidget->GetInteractor (), mv_PointCloudStreamQVTKWidget->GetRenderWindow ());
}

void TDK_ScanWindow::mf_SetupSensorWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Sensor Parameters"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    std::map<QString, QString> sensorNames = mv_SensorController->mf_GetAvailableSensorNames();
    std::map<QString, QString>::iterator it = sensorNames.begin();

    mv_SensorComboBox->setFixedHeight(22);

    while(it != sensorNames.end()){
        mv_SensorComboBox->addItem(it->second, it->first);
        it++;
    }

    mv_XMinimumSpinBox->setRange(-10, 10);
    mv_XMinimumSpinBox->setSingleStep(0.02);
    mv_XMinimumSpinBox->setValue(-0.5);
    mv_XMinimumSpinBox->setFixedHeight(22);
    mv_XMinimumSpinBox->setSuffix(QString("m"));

    mv_XMaximumSpinBox->setRange(-10, 10);
    mv_XMaximumSpinBox->setSingleStep(0.02);
    mv_XMaximumSpinBox->setValue(0.5);
    mv_XMaximumSpinBox->setFixedHeight(22);
    mv_XMaximumSpinBox->setSuffix(QString("m"));

    mv_YMinimumSpinBox->setRange(-10, 10);
    mv_YMinimumSpinBox->setSingleStep(0.02);
    mv_YMinimumSpinBox->setValue(-1.5);
    mv_YMinimumSpinBox->setFixedHeight(22);
    mv_YMinimumSpinBox->setSuffix(QString("m"));

    mv_YMaximumSpinBox->setRange(-10, 10);
    mv_YMaximumSpinBox->setSingleStep(0.02);
    mv_YMaximumSpinBox->setValue(1);
    mv_YMaximumSpinBox->setFixedHeight(22);
    mv_YMaximumSpinBox->setSuffix(QString("m"));

    mv_ZMinimumSpinBox->setRange(-10, 10);
    mv_ZMinimumSpinBox->setSingleStep(0.02);
    mv_ZMinimumSpinBox->setValue(2);
    mv_ZMinimumSpinBox->setFixedHeight(22);
    mv_ZMinimumSpinBox->setSuffix(QString("m"));

    mv_ZMaximumSpinBox->setRange(-10, 10);
    mv_ZMaximumSpinBox->setSingleStep(0.02);
    mv_ZMaximumSpinBox->setValue(3);
    mv_ZMaximumSpinBox->setFixedHeight(22);
    mv_ZMaximumSpinBox->setSuffix(QString("m"));

    mv_InclinationSpinBox->setRange(-45, 45);
    mv_InclinationSpinBox->setSingleStep(0.5);
    mv_InclinationSpinBox->setValue(0);
    mv_InclinationSpinBox->setFixedHeight(22);
    mv_InclinationSpinBox->setSuffix(QString("º"));

    mv_CapturePointCloudPushButton->setFixedHeight(22);
    mv_CapturePointCloudPushButton->setEnabled(false);

    mv_StartScanPushButton->setFixedHeight(22);
    mv_StopScanPushButton->setFixedHeight(22);

    mv_FilterBoxCheckBox->setText(QString("Activate filtering point cloud"));
    mv_RegistrationCheckBox->setText(QString("Register point cloud during scan"));

    gridLayout->addWidget(new QLabel("Select sensor : "), 0, 0, 1, 2);
    gridLayout->addWidget(mv_SensorComboBox, 0, 2, 1, 2);
    gridLayout->addWidget(new QLabel(QString("x-minimum : ")), 1, 0);
    gridLayout->addWidget(mv_XMinimumSpinBox, 1, 1);
    gridLayout->addWidget(new QLabel(QString("x-maximum : ")), 1, 2);
    gridLayout->addWidget(mv_XMaximumSpinBox, 1, 3);
    gridLayout->addWidget(new QLabel(QString("y-minimum : ")), 2, 0);
    gridLayout->addWidget(mv_YMinimumSpinBox, 2, 1);
    gridLayout->addWidget(new QLabel(QString("y-maximum : ")), 2, 2);
    gridLayout->addWidget(mv_YMaximumSpinBox, 2, 3);
    gridLayout->addWidget(new QLabel(QString("z-minimum : ")), 3, 0);
    gridLayout->addWidget(mv_ZMinimumSpinBox, 3, 1);
    gridLayout->addWidget(new QLabel(QString("z-maximum : ")), 3, 2);
    gridLayout->addWidget(mv_ZMaximumSpinBox, 3, 3);
    gridLayout->addWidget(new QLabel(QString("Inclination : ")), 4, 0);
    gridLayout->addWidget(mv_InclinationSpinBox, 4, 1);
    gridLayout->addWidget(mv_FilterBoxCheckBox, 5, 0, 1, 4);
    gridLayout->addWidget(mv_RegistrationCheckBox, 6, 0, 1, 4);
    gridLayout->addWidget(mv_StartScanPushButton, 7, 0, 1, 2);
    gridLayout->addWidget(mv_StopScanPushButton, 7, 2, 1, 2);
    gridLayout->addWidget(new QLabel(QString("Number of point clouds captured : ")), 8, 0, 1, 3);
    gridLayout->addWidget(mv_NumberOfPointCloudsCapturedLabel, 8, 3, 1, 1);
    gridLayout->addWidget(mv_CapturePointCloudPushButton, 9, 0, 1, 4);

    gridLayout->setRowMinimumHeight(0, 30);
    gridLayout->setHorizontalSpacing(10);
    gridLayout->setVerticalSpacing(20);
    gridLayout->setMargin(12);

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);

    mv_CentralGridLayout->addWidget(dockWidget, 0, 0);

}


void TDK_ScanWindow::mf_SetupPlatformParametersWidget()
{
    QDockWidget *dockWidget = new QDockWidget(tr("Platform Parameters"));
    QGridLayout *gridLayout = new QGridLayout;
    QScrollArea *scrollArea = new QScrollArea;
    QWidget *widget = new QWidget;

    mv_PlatformParametersYesRadioButton->setAutoExclusive(true);
    mv_PlatformParametersYesRadioButton->setFixedHeight(22);
    mv_PlatformParametersNoRadioButton->setAutoExclusive(true);
    mv_PlatformParametersNoRadioButton->setChecked(true);
    mv_PlatformParametersYesRadioButton->setFixedHeight(22);

    mv_IncrementalRotationAngleSpinBox->setRange(5, 360);
    mv_IncrementalRotationAngleSpinBox->setSingleStep(5);
    mv_IncrementalRotationAngleSpinBox->setValue(10);
    mv_IncrementalRotationAngleSpinBox->setFixedHeight(22);
    mv_IncrementalRotationAngleSpinBox->setSuffix(QString("°"));
    mv_IncrementalRotationAngleSpinBox->setEnabled(false);

    mv_NumberOfRotationsSpinBox->setRange(1, 10);
    mv_NumberOfRotationsSpinBox->setSingleStep(1);
    mv_NumberOfRotationsSpinBox->setValue(1);
    mv_NumberOfRotationsSpinBox->setFixedHeight(22);
    mv_NumberOfRotationsSpinBox->setEnabled(false);

    mv_SerialPortBaudRateComboBox->addItem(QString("4800"));
    mv_SerialPortBaudRateComboBox->setEnabled(false);

    mv_SerialPortNameLineEdit->setText("COM3");
    mv_SerialPortNameLineEdit->setEnabled(false);

    gridLayout->addWidget(new QLabel("Enable platform parameters : "), 0, 0, 1, 2);
    gridLayout->addWidget(mv_PlatformParametersYesRadioButton, 0, 2, 1, 1);
    gridLayout->addWidget(mv_PlatformParametersNoRadioButton, 0, 3, 1, 1);
    gridLayout->addWidget(new QLabel("Incremental rotation angle : "), 1, 0, 1, 2);
    gridLayout->addWidget(mv_IncrementalRotationAngleSpinBox, 1, 2, 1, 2);
    gridLayout->addWidget(new QLabel("Number of rotations : "), 2, 0, 1, 2);
    gridLayout->addWidget(mv_NumberOfRotationsSpinBox, 2, 2, 1, 2);
    gridLayout->addWidget(new QLabel("Serial port name : "), 3, 0, 1, 2);
    gridLayout->addWidget(mv_SerialPortNameLineEdit, 3, 2, 1, 2);
    gridLayout->addWidget(new QLabel("Serial port baud rate : "), 4, 0, 1, 2);
    gridLayout->addWidget(mv_SerialPortBaudRateComboBox, 4, 2, 1, 2);

    gridLayout->setRowMinimumHeight(0, 30);
    gridLayout->setHorizontalSpacing(10);
    gridLayout->setVerticalSpacing(20);
    gridLayout->setMargin(12);

    widget->setLayout(gridLayout);
    scrollArea->setWidget(widget);
    dockWidget->setWidget(scrollArea);
    dockWidget->setFeatures(QDockWidget::NoDockWidgetFeatures);
    mv_CentralGridLayout->addWidget(dockWidget, 1, 0);
}

void TDK_ScanWindow::mf_SlotUpdatePointCloudStream()
{
    if( !mv_PointCloudStreamVisualizer->updatePointCloud( mv_Sensor->mf_GetMvPointCloud(), "cloud" ) ){
        mv_PointCloudStreamVisualizer->addPointCloud( mv_Sensor->mf_GetMvPointCloud(), "cloud" );
        mv_PointCloudStreamVisualizer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    }
    mv_FlagPointCloudExists = true;
    mv_PointCloudStreamQVTKWidget->update();
}

void TDK_ScanWindow::mf_SlotCapturePointCloud(int degreesRotated)
{
    qDebug() << "Trying :Capturing point cloud";
    if(mv_FlagScanning && mv_FlagPointCloudExists){
        qDebug() << "Point cloud captured " << mv_Sensor->mf_GetMvPointCloud()->points.size();
        mf_SetNumberOfPointCloudsCaptured(mf_GetNumberOfPointCloudsCaptured() + 1);
        TDK_Database::mf_StaticAddPointCloud(mv_Sensor->mf_GetMvPointCloud()->makeShared());
        emit mf_SignalDatabasePointCloudUpdated();

        qDebug() << "Register";
        mv_ScanRegistration->addNextPointCloud(TDK_Database::mv_PointCloudsVector[TDK_Database::mv_PointCloudsVector.size()-1], degreesRotated);
    }
}

void TDK_ScanWindow::mf_SlotCapturePointCloudButtonClick()
{
    qDebug() << "Clicked" << mv_FlagScanning << mv_FlagPointCloudExists << !mv_FlagTurnTableParametersEnabled;
    if(mv_FlagScanning && mv_FlagPointCloudExists && !mv_FlagTurnTableParametersEnabled){
        mf_SetNumberOfPointCloudsCaptured(mf_GetNumberOfPointCloudsCaptured() + 1);
        TDK_Database::mf_StaticAddPointCloud(mv_Sensor->mf_GetMvPointCloud()->makeShared());
        emit mf_SignalDatabasePointCloudUpdated();

        qDebug() << "Register";
        mv_ScanRegistration->addNextPointCloud(TDK_Database::mv_PointCloudsVector[TDK_Database::mv_PointCloudsVector.size()-1], 0);
    }
}

void TDK_ScanWindow::mf_SlotUpdateStatusBar(QString status, QColor statusColor)
{
    QPalette statusBarPalette;
    statusBarPalette.setColor( QPalette::WindowText, statusColor );
    mv_StatusBar->setPalette(statusBarPalette);
    if(statusColor == Qt::blue){
        mv_StatusBar->showMessage(status);
    }
    else{
        mv_StatusBar->showMessage(status, 5000);
    }
}

void TDK_ScanWindow::mf_SlotUpdateBoundingBox()
{
    mv_Sensor->mf_SetFilterBox(mv_XMinimumSpinBox->value(), mv_XMaximumSpinBox->value(), mv_YMinimumSpinBox->value(), mv_YMaximumSpinBox->value(), mv_ZMinimumSpinBox->value(), mv_ZMaximumSpinBox->value());

    mv_PointCloudStreamVisualizer->removeShape("cube");
    mv_PointCloudStreamVisualizer->removeShape("rot_axis");

    mv_PointCloudStreamVisualizer->addCube(mv_XMinimumSpinBox->value(), mv_XMaximumSpinBox->value(), mv_YMinimumSpinBox->value(), mv_YMaximumSpinBox->value(), mv_ZMinimumSpinBox->value(), mv_ZMaximumSpinBox->value());
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.33, "cube");
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.0, 0.0, "cube");
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "cube");

    //Add turntableAxisOfRotation
    float x_coord = mv_XMinimumSpinBox->value() + (mv_XMaximumSpinBox->value() - mv_XMinimumSpinBox->value())/2;
    float z_coord = mv_ZMinimumSpinBox->value() + (mv_ZMaximumSpinBox->value() - mv_ZMinimumSpinBox->value())/2;

    float inclinationDegrees = mv_InclinationSpinBox->value();
    float top_z_coord = z_coord - (mv_YMaximumSpinBox->value() - mv_YMinimumSpinBox->value())*tan(inclinationDegrees*M_PI/180.0);
    float bottom_z_coord = z_coord;

    pcl::PointXYZ top(x_coord, mv_YMaximumSpinBox->value(), top_z_coord);
    pcl::PointXYZ bottom(x_coord, mv_YMinimumSpinBox->value(), bottom_z_coord);

    mv_PointCloudStreamVisualizer->addLine<pcl::PointXYZ>(bottom, top, 0.0, 0.9, 0.0, "rot_axis");
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "rot_axis");
    mv_PointCloudStreamVisualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "rot_axis");

    mv_PointCloudStreamQVTKWidget->update();
}

void TDK_ScanWindow::mf_SlotActivateFiltering(bool flagFiltering)
{
    mv_Sensor->mf_SetMvFlagFilterPoints(flagFiltering);
}

void TDK_ScanWindow::mf_SlotPointCloudRegistration(bool flagRealTimeScan)
{
    if(!mv_FlagScanning){
        mv_FlagRealTimeScan = flagRealTimeScan;
        mv_ScanRegistration->setRegisterInRealTime(flagRealTimeScan);
    }
}

void TDK_ScanWindow::mf_SlotStartScan()
{
    if(!mv_FlagScanning){
        emit mf_SignalStatusChanged(QString("Scanning..."), Qt::blue);
        mv_FlagScanning = true;
        mf_SetNumberOfPointCloudsCaptured(0);
        mv_CapturePointCloudPushButton->setEnabled(true);
        mv_SensorComboBox->setEnabled(false);
        mv_XMinimumSpinBox->setEnabled(false);
        mv_XMaximumSpinBox->setEnabled(false);
        mv_YMinimumSpinBox->setEnabled(false);
        mv_YMaximumSpinBox->setEnabled(false);
        mv_ZMinimumSpinBox->setEnabled(false);
        mv_ZMaximumSpinBox->setEnabled(false);
        mv_InclinationSpinBox->setEnabled(false);
        mv_FilterBoxCheckBox->setEnabled(false);
        mv_RegistrationCheckBox->setEnabled(false);
        mv_StartScanPushButton->setEnabled(false);
        mv_FlagPointCloudExists = false;
        qDebug() << mv_FlagTurnTableParametersEnabled << !mv_Turntable->mf_IsRunning();
        if(mv_FlagTurnTableParametersEnabled && !mv_Turntable->mf_IsRunning()){
            qDebug() << "Start platform from scan window";
            mf_InitializeScannerCenter();
            mv_Turntable->mf_SetStepAngle((int)mv_IncrementalRotationAngleSpinBox->value());
            mv_Turntable->mf_SetTotalRotations((int)mv_NumberOfRotationsSpinBox->value());
            mv_Turntable->mf_StartPlatform(mv_SerialPortNameLineEdit->text(), mv_SerialPortBaudRateComboBox->currentText().toInt());
        }
    }
}

void TDK_ScanWindow::mf_SlotStopScan()
{
    if(mv_FlagScanning){
        emit mf_SignalStatusChanged(QString("Scan stopped."), Qt::red);
        mv_FlagScanning = false;
        mv_CapturePointCloudPushButton->setEnabled(false);
        mv_SensorComboBox->setEnabled(true);
        mv_XMinimumSpinBox->setEnabled(true);
        mv_XMaximumSpinBox->setEnabled(true);
        mv_YMinimumSpinBox->setEnabled(true);
        mv_YMaximumSpinBox->setEnabled(true);
        mv_ZMinimumSpinBox->setEnabled(true);
        mv_ZMaximumSpinBox->setEnabled(true);
        mv_InclinationSpinBox->setEnabled(true);
        mv_FilterBoxCheckBox->setEnabled(true);
        mv_RegistrationCheckBox->setEnabled(true);
        mv_StartScanPushButton->setEnabled(true);

        if(mv_Turntable->mf_IsRunning()){
            mv_Turntable->mf_StopPlatform();
        }

        if(mv_FlagPointCloudExists){
            emit mf_SignalStatusChanged(QString("Registering point clouds..."), Qt::blue);
            TDK_Database::mf_StaticAddRegisteredPointCloud(mv_ScanRegistration->postProcess_and_getAlignedPC()->makeShared());
            emit mf_SignalDatabaseRegisteredPointCloudUpdated();
            emit mf_SignalStatusChanged(QString("Registration done."), Qt::green);
        }
        //mf_SetNumberOfPointCloudsCaptured(0);
        //this->close();

    }
}

void TDK_ScanWindow::mf_SlotHandlePlatformParameters(bool flagEnablePlatformParameters)
{
    mv_FlagTurnTableParametersEnabled = flagEnablePlatformParameters;
    if(flagEnablePlatformParameters){

        mv_IncrementalRotationAngleSpinBox->setEnabled(true);
        mv_NumberOfRotationsSpinBox->setEnabled(true);
        mv_SerialPortNameLineEdit->setEnabled(true);
        mv_SerialPortBaudRateComboBox->setEnabled(true);
    }
    else{
        mv_IncrementalRotationAngleSpinBox->setEnabled(false);
        mv_NumberOfRotationsSpinBox->setEnabled(false);
        mv_SerialPortNameLineEdit->setEnabled(false);
        mv_SerialPortBaudRateComboBox->setEnabled(false);
    }
}

void TDK_ScanWindow::mf_SlotUpdateWindow(int sensorIndex)
{
    if(mv_Sensor != nullptr){
        qDebug() << "Disconnecting sensor slot";
        disconnect(mv_Sensor, SIGNAL(mf_SignalPointCloudUpdated()), this, SLOT(mf_SlotUpdatePointCloudStream()));
    }
    qDebug() << mv_SensorComboBox->count();
    QString sensorName = mv_SensorComboBox->itemText(sensorIndex);
    qDebug() << "Getting sensor in ui";
    mv_Sensor = mv_SensorController->mf_GetSensor(sensorName);
    qDebug() << mv_Sensor->mf_GetMvName();
    mv_Sensor->mf_SetFilterBox(mv_XMinimumSpinBox->value(), mv_XMaximumSpinBox->value(), mv_YMinimumSpinBox->value(), mv_YMaximumSpinBox->value(), mv_ZMinimumSpinBox->value(), mv_ZMaximumSpinBox->value());
    mv_Sensor->mf_SetMvFlagFilterPoints(mv_FilterBoxCheckBox->isChecked());
    connect(mv_Sensor, SIGNAL(mf_SignalPointCloudUpdated()), this, SLOT(mf_SlotUpdatePointCloudStream()));
    mv_Sensor->mf_StartSensor();
}

int TDK_ScanWindow::mf_GetNumberOfPointCloudsCaptured() const
{
    return mv_NumberOfPointCloudsCaptured;
}

void TDK_ScanWindow::mf_SetNumberOfPointCloudsCaptured(int value)
{
    mv_NumberOfPointCloudsCaptured = value;
    emit mf_SignalNumberOfPointCloudUpdated(value);
}

void TDK_ScanWindow::mf_InitializeScannerCenter()
{
    float x_coord = mv_XMinimumSpinBox->value() + (mv_XMaximumSpinBox->value() - mv_XMinimumSpinBox->value())/2;
    float z_coord = mv_ZMinimumSpinBox->value() + (mv_ZMaximumSpinBox->value() - mv_ZMinimumSpinBox->value())/2;
    float inclinationDegrees = mv_InclinationSpinBox->value();

    mv_ScannerCenter.x = x_coord;
    mv_ScannerCenter.y = mv_YMinimumSpinBox->value();
    mv_ScannerCenter.z = z_coord;
    mv_ScannerCenter.vp_x = inclinationDegrees;
    mv_ScannerCenter.vp_y = 0.0;
    mv_ScannerCenter.vp_z = 0.0;

    mv_ScanRegistration->setScannerRotationAxis(mv_ScannerCenter);
}
