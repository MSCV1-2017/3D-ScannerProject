#ifndef TDK_SCANWINDOW_H
#define TDK_SCANWINDOW_H

// Include Qt headers
#include <QMainWindow>
#include <QStatusBar>
#include <QGridLayout>
#include <QDockWidget>
#include <QScrollArea>
#include <QLabel>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <map>
#include <QDebug>
#include <QRadioButton>
#include <QKeyEvent>
#include <QLineEdit>

//Include PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>

#include "tdk_sensorcontroller.h"
#include "tdk_database.h"
#include "tdk_scanregistration.h"
#include "tdk_turntable.h"

class TDK_ScanWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit TDK_ScanWindow(QMainWindow *parent = 0);
    ~TDK_ScanWindow();

    QWidget                                             *mv_CentralWidget;
    QStatusBar                                          *mv_StatusBar;
    QGridLayout                                         *mv_CentralGridLayout;
    TDK_SensorController                                *mv_SensorController;
    TDK_Sensor                                          *mv_Sensor;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> mv_PointCloudStreamVisualizer;
    QVTKWidget                                          *mv_PointCloudStreamQVTKWidget;
    int                                                  mv_NumberOfPointCloudsCaptured;
    TDK_ScanRegistration                                *mv_ScanRegistration;
    TDK_Turntable                                       *mv_Turntable;


    //Flag variables
    bool        mv_FlagRealTimeScan;
    bool        mv_FlagScanning;
    bool        mv_FlagTurnTableParametersEnabled;
    bool        mv_FlagPointCloudExists;

    //Sensor widgets
    QComboBox           *mv_SensorComboBox;
    QDoubleSpinBox      *mv_XMinimumSpinBox;
    QDoubleSpinBox      *mv_XMaximumSpinBox;
    QDoubleSpinBox      *mv_YMinimumSpinBox;
    QDoubleSpinBox      *mv_YMaximumSpinBox;
    QDoubleSpinBox      *mv_ZMinimumSpinBox;
    QDoubleSpinBox      *mv_ZMaximumSpinBox;
    QDoubleSpinBox      *mv_InclinationSpinBox;
    QCheckBox           *mv_FilterBoxCheckBox;
    QCheckBox           *mv_RegistrationCheckBox;
    QPushButton         *mv_StartScanPushButton;
    QPushButton         *mv_StopScanPushButton;
    QLabel              *mv_NumberOfPointCloudsCapturedLabel;
    QPushButton         *mv_CapturePointCloudPushButton;

    //Platform parameters widgets
    QRadioButton                *mv_PlatformParametersYesRadioButton;
    QRadioButton                *mv_PlatformParametersNoRadioButton;
    QDoubleSpinBox              *mv_IncrementalRotationAngleSpinBox;
    QDoubleSpinBox              *mv_NumberOfRotationsSpinBox;
    QLineEdit                   *mv_SerialPortNameLineEdit;
    QComboBox                   *mv_SerialPortBaudRateComboBox;
    pcl::PointWithViewpoint      mv_ScannerCenter;

    void    mf_setupUI                                  ();
    void    mf_SetupPointCloudStreamWidget              ();
    void    mf_SetupSensorWidget                        ();
    void    mf_SetupPlatformParametersWidget            ();

    int     mf_GetNumberOfPointCloudsCaptured           ()                          const;
    void    mf_SetNumberOfPointCloudsCaptured           (int value);

    void    mf_InitializeScannerCenter                  ();

signals:
    void    mf_SignalStatusChanged                      (QString, QColor);
    void    mf_SignalDatabasePointCloudUpdated          ();
    void    mf_SignalDatabaseRegisteredPointCloudUpdated();
    void    mf_SignalNumberOfPointCloudUpdated          (int);


public slots:
    void    mf_SlotUpdateWindow                         (int sensorIndex);
    void    mf_SlotUpdateBoundingBox                    ();
    void    mf_SlotActivateFiltering                    (bool flagFiltering);
    void    mf_SlotPointCloudRegistration               (bool flagRealTimeScan);
    void    mf_SlotStartScan                            ();
    void    mf_SlotStopScan                             ();

    void    mf_SlotHandlePlatformParameters             (bool flagEnablePlatformParameters);

    void    mf_SlotUpdatePointCloudStream               ();
    void    mf_SlotCapturePointCloud                    (int degreesRotated);
    void    mf_SlotCapturePointCloudButtonClick         ();

    void    mf_SlotUpdateStatusBar                      (QString status, QColor statusColor);

};

#endif // TDK_SCANWINDOW_H
