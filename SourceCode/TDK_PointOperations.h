#ifndef PCOPERATIONS_H
#define PCOPERATIONS_H


#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
//marching cube
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/marching_cubes.h>
//#include <pcl/surface/marching_cubes_greedy.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/concave_hull.h>


#include <vtkVersion.h>
#include <vtkSmoothPolyDataFilter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/processing.h>
#include <pcl/surface/vtk_smoothing/vtk.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/marching_cubes.h>


#include <QDebug>





using namespace pcl;

class TDK_PointOperations
{
public:
    TDK_PointOperations();
    //function for trying class -> i dont use it right now
    static void FilterPCPassthrough(const double &ci, const double &minx, const double &maxx, const double &miny, const double &maxy, const double &minz, const double &maxz, uint &xi, uint &yi, uint &zi, const PointCloud<PointXYZ>::Ptr &input, PointCloud<PointXYZ>::Ptr &output);

    //new passthrough filter

    static void mf_NormalEstimation(PointCloud<PointXYZ>::Ptr &mv_PointCloudInput, PointCloud<pcl::PointNormal>::Ptr &mv_PointNormalOutput);

    //marching cube


    //for input of type PointXYZRGB
    static void mf_PoissonMeshesWithConversion(const PointCloud<PointXYZRGB>::Ptr &mv_PointCloudInput , PolygonMesh::Ptr &mv_MeshesOutput);

    //for input of type PointXYZ
    static void mf_PoissonMeshes(const PointCloud<PointXYZ>::Ptr &mv_PointCloudInput , PolygonMesh::Ptr &mv_MeshesOutput);
    static void mf_ConvertFromXYZRGBtoXYZ(const PointCloud<pcl::PointXYZRGB>::Ptr &mv_PointCloudInput, PointCloud<pcl::PointXYZ>::Ptr &mv_PointCloudOutput);

    static void mf_TriangulationMeshes(const PointCloud<PointXYZ>::Ptr &mv_PointCloudInput, pcl::PolygonMesh::Ptr &mv_MeshesOutput);
};

#endif // TDK_POINTOPERATIONS_H
