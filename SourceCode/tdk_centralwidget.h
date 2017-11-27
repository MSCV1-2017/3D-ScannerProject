#ifndef TDK_CENTRALWIDGET_H
#define TDK_CENTRALWIDGET_H

//Include QT libraries
#include <QWidget>
#include <QGridLayout>
#include <QDockWidget>
#include <QScrollArea>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QVTKWidget.h>
#include <QListWidget>
#include <QMessageBox>
#include <QDebug>

//Include PCL IO libraries
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//Include PCL visualizer
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

//Include custom classes
#include "tdk_database.h"
#include "tdk_scanregistration.h"
#include "TDK_PointOperations.h"

class TDK_CentralWidget : public QWidget
{
    Q_OBJECT
public:
    explicit TDK_CentralWidget(QWidget *parent = 0);
    ~TDK_CentralWidget();

    QGridLayout          *mv_CentralGridLayout;
    QVTKWidget           *mv_PointCloudQVTKWidget;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> mv_PointCloudVisualizer;

    //Point cloud operations
    QComboBox            *mv_MeshAlgorithmComboBox;
    QPushButton          *mv_GenerateMeshPushButton;
    QComboBox            *mv_RegistrationComboBox;
    QPushButton          *mv_RegistrationPushButton;
    TDK_ScanRegistration *mv_ScanRegistration;

    //Explorer widget
    QTabWidget           *mv_PointCloudExplorerTabWidget;
    QListWidget          *mv_PointCloudListTab;
    QListWidget          *mv_RegisteredPointCloudListTab;
    QListWidget          *mv_MeshListTab;
    unsigned int          mv_numberOfPointCloudsSelected;
    unsigned int          mv_numberOfMeshesSelected;

    //Setup widget functions
    void    mf_setupUI                              ();
    void    mf_SetupPointCloudDisplayWidget         ();
    void    mf_SetupPointCloudExplorerTabWidget     ();
    void    mf_SetupPointCloudOperationsWidget      ();

signals:
    //Signals when pointclouds and meshes are updated
    void    mf_SignalPointCloudListUpdated          ();
    void    mf_SignalRegisteredPointCloudListUpdated();
    void    mf_SignalMeshListUpdated                ();

public slots:
    //Slots to handle register point cloud and generate mesh
    void    mf_SlotRegisterPointCloud               ();
    void    mf_SlotGenerateMesh                     ();

    //Slots to update pointcloud and mesh list tab
    void    mf_SlotUpdatePointCloudListTab          ();
    void    mf_SlotUpdateRegisteredPointCloudListTab();
    void    mf_SlotUpdateMeshListTab                ();

    //Slots to display pointclouds and meshes selected in list tab
    void    mf_SlotUpdatePointCloudDisplay          (QListWidgetItem* item);
    void    mf_SlotUpdateRegisteredPointCloudDisplay(QListWidgetItem* item);
    void    mf_SlotUpdateMeshDisplay                (QListWidgetItem* item);
};

#endif // TDK_CENTRALWIDGET_H
