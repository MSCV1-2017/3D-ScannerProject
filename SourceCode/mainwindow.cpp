#include "mainwindow.h"
#include "ui_mainwindow.h"

/***************************************************************************
 * Input argument(s) : QWidget *parent - Parent class pointer
 * Return type       : NA
 * Functionality     : Constructor to initialize variables
 *
 **************************************************************************/
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
    ui              (new Ui::MainWindow),
    mv_ScanWindow   (new TDK_ScanWindow(this))
{

    ui->setupUi(this);

    //Connection to update pointcloud and registered pointcloud list after scan from scan window
    connect(mv_ScanWindow           , SIGNAL(mf_SignalDatabasePointCloudUpdated()),
            ui->centralWidget       , SLOT(mf_SlotUpdatePointCloudListTab()));
    connect(mv_ScanWindow           , SIGNAL(mf_SignalDatabaseRegisteredPointCloudUpdated()),
            ui->centralWidget       , SLOT(mf_SlotUpdateRegisteredPointCloudListTab()));

    //Connection to update pointcloud and mesh list after import from main window
    connect(this                    , SIGNAL(mf_SignalDatabasePointCloudUpdated()),
            ui->centralWidget       , SLOT(mf_SlotUpdatePointCloudListTab()));
    connect(this                    , SIGNAL(mf_SignalDatabaseMeshUpdated()),
            ui->centralWidget       , SLOT(mf_SlotUpdateMeshListTab()));

}

/***************************************************************************
 * Input argument(s) : NA
 * Return type       : NA
 * Functionality     : Destructor to free variables
 *
 **************************************************************************/
MainWindow::~MainWindow()
{
    delete ui;
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to handle new scan action.
 *
 **************************************************************************/
void MainWindow::on_actionNew_Scan_triggered()
{
    mv_ScanWindow->mv_SensorController->mf_InitializeSensors();
    if(mv_ScanWindow->mv_SensorController->mf_IsSensorAvailable()){
        mv_ScanWindow->mf_setupUI();
        mv_ScanWindow->setWindowTitle("3D KORN SCANNER - SCAN WINDOW");
        mv_ScanWindow->showMaximized();
    }
    else{
        bool retryFlag = true;
        while(retryFlag){
            QMessageBox sensorWarningMessageBox;
            sensorWarningMessageBox.setIcon(QMessageBox::Warning);
            sensorWarningMessageBox.setText("No sensor connected. Please connect sensor and click retry.");
            sensorWarningMessageBox.setStandardButtons(QMessageBox::Retry | QMessageBox::Cancel);
            sensorWarningMessageBox.setDefaultButton(QMessageBox::Retry);
            int retryValue = sensorWarningMessageBox.exec();
            switch (retryValue) {

            case QMessageBox::Retry:
                if(mv_ScanWindow->mv_SensorController->mf_IsSensorAvailable()){
                    mv_ScanWindow->mf_setupUI();
                    mv_ScanWindow->setWindowTitle("3D KORN SCANNER - SCAN WINDOW");
                    mv_ScanWindow->showMaximized();
                    retryFlag = false;
                }
                break;

            case QMessageBox::Cancel:
                retryFlag = false;
                break;

            default:
                retryFlag = false;
                break;
            }
        }
    }
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to handle import pointcloud action.
 *
 **************************************************************************/
void MainWindow::on_actionImportPointCloud_triggered()
{
    QStringList pointCloudFileNamesList = QFileDialog::getOpenFileNames(this, QString("Import point clouds"), QString(""), QString("Point Cloud (*.pcd *.ply)"));
    if(pointCloudFileNamesList.size() !=0){
        for(int i = 0; i < pointCloudFileNamesList.size(); i++){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>());
            //Load ply files
            if(pointCloudFileNamesList[i].endsWith(".ply")){
                pcl::io::loadPLYFile(pointCloudFileNamesList[i].toStdString(), *pointCloudPtr);
                TDK_Database::mf_StaticAddPointCloud(pointCloudPtr);
                emit mf_SignalDatabasePointCloudUpdated();
            }
            //Load pcd files
            else if(pointCloudFileNamesList[i].endsWith(".pcd")){
                pcl::io::loadPCDFile(pointCloudFileNamesList[i].toStdString(), *pointCloudPtr);
                TDK_Database::mf_StaticAddPointCloud(pointCloudPtr);
                emit mf_SignalDatabasePointCloudUpdated();
            }

        }
    }
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to handle import mesh action.
 *
 **************************************************************************/
void MainWindow::on_actionImportMesh_triggered()
{
    QStringList meshFileNamesList = QFileDialog::getOpenFileNames(this, QString("Import mesh"), QString(""), QString("Mesh (*.vtk *.stl)"));
    if(meshFileNamesList.size() !=0){
        for(int i = 0; i < meshFileNamesList.size(); i++){
            pcl::PolygonMesh::Ptr meshPtr (new pcl::PolygonMesh());

            //Load stl files
            if(meshFileNamesList[i].endsWith(".stl")){
                pcl::io::loadPolygonFileSTL(meshFileNamesList[i].toStdString(), *meshPtr);
                TDK_Database::mf_StaticAddMesh(meshPtr);
                emit mf_SignalDatabaseMeshUpdated();
            }

            //Load vtk files
            else if(meshFileNamesList[i].endsWith(".vtk")){
                pcl::io::loadPolygonFileVTK(meshFileNamesList[i].toStdString(), *meshPtr);
                TDK_Database::mf_StaticAddMesh(meshPtr);
                emit mf_SignalDatabaseMeshUpdated();
            }
        }
    }
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to handle export pcd action.
 *
 **************************************************************************/
void MainWindow::on_actionExportPCD_triggered()
{
    TDK_CentralWidget* centralwidget = (TDK_CentralWidget*)centralWidget();
    if(centralwidget->mv_numberOfPointCloudsSelected == 0){
        QMessageBox::warning(this, QString("3D-KORN"), QString("Please select one or more point clouds to export."));
    }
    else{
        QString directoryName = QFileDialog::getExistingDirectory(this, QString("Export point cloud"),QString(""), QFileDialog::ShowDirsOnly);
        if(directoryName != ""){
            QString filePath;
            QListWidgetItem* item;

            //Save pointclouds selected in pointcloud tab
            for(int i = 0, len = centralwidget->mv_PointCloudListTab->count(); i < len; i++)
            {
                item = centralwidget->mv_PointCloudListTab->item(i);
                if(item->checkState() == Qt::Checked)
                {
                    filePath = directoryName + "/" + item->text() + ".pcd";
                    qDebug() << filePath << " : pointcloud size - " << TDK_Database::mv_PointCloudsVector[i]->points.size();
                    pcl::io::savePCDFile(filePath.toStdString(), *(TDK_Database::mv_PointCloudsVector[i]));
                }
            }

            //Save pointclouds selected in registered pointcloud tab
            for(int i = 0, len = centralwidget->mv_RegisteredPointCloudListTab->count(); i < len; i++)
            {
                item = centralwidget->mv_RegisteredPointCloudListTab->item(i);
                if(item->checkState() == Qt::Checked)
                {
                    filePath = directoryName + "/" + item->text() + ".pcd";
                    qDebug() << filePath;
                    pcl::io::savePCDFile(filePath.toStdString(), *(TDK_Database::mv_RegisteredPointCloudsVector[i]));
                }
            }
        }
    }
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to handle export ply action.
 *
 **************************************************************************/
void MainWindow::on_actionExportPLY_triggered()
{
    TDK_CentralWidget* centralwidget = (TDK_CentralWidget*)centralWidget();
    if(centralwidget->mv_numberOfPointCloudsSelected == 0){
        QMessageBox::warning(this, QString("3D-KORN"), QString("Please select one or more point clouds to export."));
    }
    else{
        QString directoryName = QFileDialog::getExistingDirectory(this, QString("Export point cloud"),QString(""), QFileDialog::ShowDirsOnly);
        if(directoryName != ""){
            QString filePath;
            QListWidgetItem* item;

            //Save pointclouds selected in pointcloud tab
            for(int i = 0, len = centralwidget->mv_PointCloudListTab->count(); i < len; i++)
            {
                item = centralwidget->mv_PointCloudListTab->item(i);
                if(item->checkState() == Qt::Checked)
                {
                    filePath = directoryName + "/" + item->text() + ".ply";
                    qDebug() << filePath;
                    pcl::io::savePLYFile(filePath.toStdString(), *(TDK_Database::mv_PointCloudsVector[i]));
                }
            }

            //Save pointclouds selected in registered pointcloud tab
            for(int i = 0, len = centralwidget->mv_RegisteredPointCloudListTab->count(); i < len; i++)
            {
                item = centralwidget->mv_RegisteredPointCloudListTab->item(i);
                if(item->checkState() == Qt::Checked)
                {
                    filePath = directoryName + "/" + item->text() + ".ply";
                    qDebug() << filePath;
                    pcl::io::savePLYFile(filePath.toStdString(), *(TDK_Database::mv_RegisteredPointCloudsVector[i]));
                }
            }
        }
    }
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to handle export stl action.
 *
 **************************************************************************/
void MainWindow::on_actionExportSTL_triggered()
{
    TDK_CentralWidget* centralwidget = (TDK_CentralWidget*)centralWidget();
    if(centralwidget->mv_numberOfMeshesSelected == 0){
        QMessageBox::warning(this, QString("3D-KORN"), QString("Please select one or more meshes to export."));
    }
    else{
        QString directoryName = QFileDialog::getExistingDirectory(this, QString("Export mesh"),QString(""), QFileDialog::ShowDirsOnly);
        if(directoryName != ""){
            QString filePath;
            QListWidgetItem* item;

            //Save meshes selected in mesh tab
            for(int i = 0, len = centralwidget->mv_MeshListTab->count(); i < len; i++)
            {
                item = centralwidget->mv_MeshListTab->item(i);
                if(item->checkState() == Qt::Checked)
                {
                    filePath = directoryName + "/" + item->text() + ".stl";
                    qDebug() << filePath;
                    pcl::io::savePolygonFileSTL(filePath.toStdString(), *(TDK_Database::mv_MeshesVector[i]));
                }
            }
        }
    }
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to handle export vtk action.
 *
 **************************************************************************/
void MainWindow::on_actionExportVTK_triggered()
{

    TDK_CentralWidget* centralwidget = (TDK_CentralWidget*)centralWidget();
    if(centralwidget->mv_numberOfMeshesSelected == 0){
        QMessageBox::warning(this, QString("3D-KORN"), QString("Please select one or more meshes to export."));
    }
    else{
        QString directoryName = QFileDialog::getExistingDirectory(this, QString("Export mesh"),QString(""), QFileDialog::ShowDirsOnly);
        if(directoryName != ""){
            QString filePath;
            QListWidgetItem* item;

            //Save meshes selected in mesh tab
            for(int i = 0, len = centralwidget->mv_MeshListTab->count(); i < len; i++)
            {
                item = centralwidget->mv_MeshListTab->item(i);
                if(item->checkState() == Qt::Checked)
                {
                    filePath = directoryName + "/" + item->text() + ".vtk";
                    qDebug() << filePath;
                    pcl::io::savePolygonFileVTK(filePath.toStdString(), *(TDK_Database::mv_MeshesVector[i]));
                }
            }
        }
    }
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Slot function to handle About action.
 *
 **************************************************************************/
void MainWindow::on_actionAbout_triggered()
{

}
