#include "TDK_PointOperations.h"


using namespace pcl;


TDK_PointOperations::TDK_PointOperations()
{


}



void TDK_PointOperations::FilterPCPassthrough(const double &ci, const double &minx, const double &maxx, const double &miny, const double &maxy, const double &minz, const double &maxz, uint &xi, uint &yi, uint &zi, const PointCloud<PointXYZ>::Ptr &cloud, PointCloud<PointXYZ>::Ptr &clouda){

    double xremapplus,xremapminus,yremapplus,yremapminus,zremapplus,zremapminus;

    xremapplus  =  maxx - xi*(maxx-minx)/255;
    xremapminus =  xi*(maxx-minx)/255+minx;

    yremapplus  =  maxy - yi*(maxy-miny)/255;
    yremapminus =  yi*(maxy-miny)/255+miny;

    zremapplus  =  maxz - zi*(maxz-minz)/255;
    zremapminus =  zi*(maxz-minz)/255+minz;
    PassThrough<PointXYZ> pass (true);

    if (ci == 1){
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (zremapplus, maxz);
        pass.setInputCloud (cloud);
        pass.filter (*clouda);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (yremapplus, maxy);
        pass.setInputCloud (clouda);
        pass.filter (*clouda);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (xremapplus, maxx); //maxx
        pass.setInputCloud (clouda);
        pass.filter (*clouda);
    }
    else{
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (minz,zremapminus);
        pass.setInputCloud (clouda);
        pass.filter (*clouda);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (miny,yremapminus);
        pass.setInputCloud (clouda);
        pass.filter (*clouda);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (minx,xremapminus);
        pass.setInputCloud (clouda);
        pass.filter (*clouda);
    }
}
//With NormalEstimationFunction we normalize the output received from the FilterPassThrough
//mf_NormalEstimation receives as input the filtered PointCloud and gives as output a PointNormal
void TDK_PointOperations::mf_NormalEstimation(PointCloud<PointXYZ>::Ptr &mv_PointCloudInput, PointCloud<pcl::PointNormal>::Ptr &mv_PointNormalOutput){

    pcl::search::KdTree<pcl::PointXYZ>::Ptr mv_Tree;
    mv_Tree.reset(new pcl::search:: KdTree<pcl::PointXYZ>(false) ) ;
    mv_Tree->setInputCloud(mv_PointCloudInput);
    cout << "begin normal estimation" << endl;
    NormalEstimationOMP<PointXYZ, Normal> mv_Normal;

    mv_Normal.setNumberOfThreads(8);
    mv_Normal.setSearchMethod(mv_Tree);
    mv_Normal.setInputCloud(mv_PointCloudInput);
    mv_Normal.setRadiusSearch(0.3);
    Eigen::Vector4f centroid;
    compute3DCentroid(*mv_PointCloudInput, centroid);
    mv_Normal.setViewPoint(centroid[0], centroid[1], centroid[2]);
    PointCloud<Normal>::Ptr mv_CloudNormals (new PointCloud<Normal>());
    mv_Normal.compute(*mv_CloudNormals);

    cout << "normal estimation complete" << endl;

    cout << "reverse normals' direction" << endl;
    for(size_t i = 0; i < mv_CloudNormals->size(); ++i){
        mv_CloudNormals->points[i].normal_x *= -1;
        mv_CloudNormals->points[i].normal_y *= -1;
        mv_CloudNormals->points[i].normal_z *= -1;
    }

    cout << "combine points and normals" << endl;
    concatenateFields(*mv_PointCloudInput, *mv_CloudNormals, *mv_PointNormalOutput);
}


//mf_PoissonMeshes takes as input a PointCloud, applies all the function defined above and gives as output a mesh
void TDK_PointOperations::mf_TriangulationMeshes(const PointCloud<PointXYZ>::Ptr &mv_PointCloudInput ,  pcl::PolygonMesh::Ptr &mv_MeshesOutput){

    //voxel filterin
    pcl::PointCloud<PointXYZ>::Ptr mv_CloudFiltered (new pcl::PointCloud<PointXYZ> ()); //cloud_filtered
    pcl::VoxelGrid<pcl::PointXYZ> mv_VoxelGrid;
    // mv_VoxelGrid.setInputCloud (mv_PointCloudForFiltering);
    mv_VoxelGrid.setInputCloud (mv_PointCloudInput);
    mv_VoxelGrid.setLeafSize (0.01f, 0.01f, 0.01f);
    //mv_VoxelGrid.setLeafSize (0.009f, 0.009f, 0.009f);
    mv_VoxelGrid.filter (*mv_CloudFiltered);


    pcl::search::KdTree<pcl::PointXYZ>::Ptr mv_Tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_PointCloud1 (new  pcl::PointCloud<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mv_MovingLeastSquares;
    mv_MovingLeastSquares.setComputeNormals (true);//need this true
    mv_MovingLeastSquares.setSearchMethod(mv_Tree1);
    mv_MovingLeastSquares.setPolynomialFit (true);
    mv_MovingLeastSquares.setSearchRadius (0.03);

    mv_MovingLeastSquares.setInputCloud (mv_CloudFiltered);
    mv_MovingLeastSquares.process (*mv_PointCloud1);


    PointCloud<PointNormal>::Ptr mv_PointNormal1(new PointCloud<PointNormal>());
    TDK_PointOperations::mf_NormalEstimation(mv_PointCloud1, mv_PointNormal1);

    //TRIANGULIZATION
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    gp3.setSearchRadius (0.8);

    // Set typical values for the parameters
    gp3.setMu (10);
    gp3.setMaximumNearestNeighbors (20);
    gp3.setMaximumSurfaceAngle(M_PI); // 45 degrees
    gp3.setMinimumAngle(M_PI/10); // 10 degrees
    gp3.setMaximumAngle(M_PI); // 120 degrees
    gp3.setNormalConsistency(false);
    // Get result
    gp3.setInputCloud (mv_PointNormal1);
    //gp3.setSearchMethod (tree2);
    pcl::PolygonMesh::Ptr mv_MeshesOutput1 (new pcl::PolygonMesh) ;
    gp3.reconstruct (*mv_MeshesOutput1);

    pcl::MeshSmoothingLaplacianVTK mv_VTK ;
    mv_VTK.setInputMesh(mv_MeshesOutput1);
    mv_VTK.setNumIter(20000);
    mv_VTK.setConvergence(0.0001);
    mv_VTK.setRelaxationFactor(0.0001);
    mv_VTK.setFeatureEdgeSmoothing(true);
    mv_VTK.setFeatureAngle(M_PI/5);
    mv_VTK.setBoundarySmoothing(true);
    mv_VTK.process(*mv_MeshesOutput);
}

void TDK_PointOperations::mf_PoissonMeshes(const PointCloud<PointXYZ>::Ptr &mv_PointCloudInput ,  pcl::PolygonMesh::Ptr &mv_MeshesOutput){

    //voxel filterin
    pcl::PointCloud<PointXYZ>::Ptr mv_CloudFiltered (new pcl::PointCloud<PointXYZ> ()); //cloud_filtered
    pcl::VoxelGrid<pcl::PointXYZ> mv_VoxelGrid;
    mv_VoxelGrid.setLeafSize (0.01f, 0.01f, 0.01f);
    mv_VoxelGrid.setInputCloud (mv_PointCloudInput);
    mv_VoxelGrid.filter (*mv_CloudFiltered);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr mv_Tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mv_PointCloud1 (new  pcl::PointCloud<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mv_MovingLeastSquares;
    mv_MovingLeastSquares.setInputCloud (mv_CloudFiltered);
    mv_MovingLeastSquares.setComputeNormals (true);//need this true
    mv_MovingLeastSquares.setSearchMethod(mv_Tree1);
    mv_MovingLeastSquares.setPolynomialFit (true);
    mv_MovingLeastSquares.setSearchRadius (0.03);
    // mv_MovingLeastSquares.setSearchRadius (0.01);
    mv_MovingLeastSquares.process (*mv_PointCloud1);

    PointCloud<PointNormal>::Ptr mv_PointNormal1(new PointCloud<PointNormal>());
    TDK_PointOperations::mf_NormalEstimation(mv_PointCloud1, mv_PointNormal1);

    //Poisson Function
    Poisson<PointNormal> mv_Poisson;
    mv_Poisson.setInputCloud(mv_PointNormal1);
    mv_Poisson.setDepth(9);
    mv_Poisson.setSolverDivide (8); //8 is the best value
    mv_Poisson.setIsoDivide (8);
    mv_Poisson.setSamplesPerNode(1);
    mv_Poisson.setConfidence(false);
    mv_Poisson.setManifold(true);
    mv_Poisson.setOutputPolygons(false);

    pcl::PolygonMesh::Ptr mv_MeshesOutput1 (new pcl::PolygonMesh) ;
    mv_Poisson.reconstruct(*mv_MeshesOutput1);
    mv_Poisson.getOutputPolygons();

    pcl::MeshSmoothingLaplacianVTK mv_VTK ;
    mv_VTK.setInputMesh(mv_MeshesOutput1);
    mv_VTK.setNumIter(20000);

    mv_VTK.setConvergence(0.0001);

    mv_VTK.setRelaxationFactor(0.0001);
    mv_VTK.setFeatureEdgeSmoothing(true);
    mv_VTK.setFeatureAngle(M_PI/5);
    mv_VTK.setBoundarySmoothing(true);
    mv_VTK.process(*mv_MeshesOutput);
}

static void mf_PoissonMeshesWithConversion(const PointCloud<PointXYZRGB>::Ptr &PointCloudXYZRGB, PolygonMesh::Ptr &mv_MeshesOutput){

    PointCloud<PointXYZ>::Ptr mv_PointCloudInput  ((new PointCloud<PointXYZ>)) ;
    TDK_PointOperations::mf_ConvertFromXYZRGBtoXYZ(PointCloudXYZRGB, mv_PointCloudInput) ;

    //Normal Estimation function
    PointCloud<PointNormal>::Ptr mv_PointNormal(new PointCloud<PointNormal>());
    TDK_PointOperations::mf_NormalEstimation(mv_PointCloudInput, mv_PointNormal);

    //Poisson Function
    Poisson<PointNormal> mv_Poisson;
    mv_Poisson.setInputCloud(mv_PointNormal);
    mv_Poisson.setDepth(10);
    mv_Poisson.setSolverDivide (8);
    mv_Poisson.setIsoDivide (8);
    mv_Poisson.setConfidence(false);
    mv_Poisson.setManifold(true);
    mv_Poisson.setOutputPolygons(false);
    //PolygonMesh;
    mv_Poisson.reconstruct(*mv_MeshesOutput);
}


void TDK_PointOperations::mf_ConvertFromXYZRGBtoXYZ(const PointCloud<pcl::PointXYZRGB>::Ptr &mv_PointCloudInput, PointCloud<pcl::PointXYZ>::Ptr &mv_PointCloudOutput){
    copyPointCloud(*mv_PointCloudInput, *mv_PointCloudOutput);
}


