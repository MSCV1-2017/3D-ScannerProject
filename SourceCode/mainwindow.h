#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//Include QT libraries
#include <QMainWindow>
#include <QMessageBox>
#include <QFileDialog>

//Include PCL libraries
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

//Include custom libraries
#include "tdk_scanwindow.h"
#include "tdk_database.h"

//Defining MainWindow in Ui namespace
namespace Ui {
class MainWindow;
}

/******************************************************************************
 * Description       : MainWindow of application
 * Author            : Software Unicorns
 *
 *****************************************************************************/
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    //Constructor and Destructor
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

signals:
    //Signals when pointcloud and mesh list is updated during import
    void    mf_SignalDatabasePointCloudUpdated      ();
    void    mf_SignalDatabaseMeshUpdated            ();

private slots:
    //Slots for menubar actions
    void    on_actionNew_Scan_triggered             ();
    void    on_actionImportPointCloud_triggered     ();
    void    on_actionImportMesh_triggered           ();
    void    on_actionExportPCD_triggered            ();
    void    on_actionExportPLY_triggered            ();
    void    on_actionExportSTL_triggered            ();
    void    on_actionExportVTK_triggered            ();
    void    on_actionAbout_triggered                ();

private:
    Ui::MainWindow      *ui;                                            //Pointer to Main window in Ui namespace
    TDK_ScanWindow      *mv_ScanWindow;                                 //Pointer to scan window
};

#endif // MAINWINDOW_H
