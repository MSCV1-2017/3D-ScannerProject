#include "tdk_sensor.h"

/***************************************************************************
 * Input argument(s) : QObject *parent - Parent class pointer
 * Return type       : NA
 * Functionality     : Constructor to initialize variables
 *
 **************************************************************************/
TDK_Sensor::TDK_Sensor(QObject *parent) : QObject(parent),
    mv_FlagFilterPoints (   false   )   ,
    mv_XMin             (   -0.5    )   ,
    mv_XMax             (    0.5    )   ,
    mv_YMin             (   -1.5    )   ,
    mv_YMax             (    1.0    )   ,
    mv_ZMin             (    2.0    )   ,
    mv_ZMax             (    3.0    )
{

}

/***************************************************************************
 * Input argument(s) : NA
 * Return type       : NA
 * Functionality     : Destructor to free variables
 *
 **************************************************************************/
TDK_Sensor::~TDK_Sensor()
{

}

/***************************************************************************
 * Input argument(s) : float xmin - Minimum x value
 *                     float xmax - Maximum x value
 *                     float ymin - Minimum y value
 *                     float xmax - Maximum y value
 *                     float zmin - Minimum z value
 *                     float zmax - Maximum z value
 * Return type       : void
 * Functionality     : Function to set new limits of filter box. Emits
 *                     signal after filter box limits are updated.
 *
 **************************************************************************/
void TDK_Sensor::mf_SetFilterBox(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
{
    mv_XMin = xmin;
    mv_XMax = xmax;
    mv_YMin = ymin;
    mv_YMax = ymax;
    mv_ZMin = zmin;
    mv_ZMax = zmax;
    emit mf_SignalFilterBoxUpdated();
}


/*****************************Setter functions*****************************/

void TDK_Sensor::mf_SetMvPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pointCloudPtr)
{
    if(pointCloudPtr != nullptr && pointCloudPtr->points.size() > 0){
        mv_PointCloud = pointCloudPtr->makeShared();
        emit mf_SignalPointCloudUpdated();
    }
}

void TDK_Sensor::mf_SetMvFlagFilterPoints(bool value)
{
    mv_FlagFilterPoints = value;
    emit mf_SignalFlagFilterUpdated();
}

/****************************************************************************/
