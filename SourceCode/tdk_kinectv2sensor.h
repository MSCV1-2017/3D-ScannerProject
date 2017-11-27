#ifndef TDK_KINECTV2SENSOR_H
#define TDK_KINECTV2SENSOR_H

#include "tdk_sensor.h"
#include "kinect2_grabber.h"

#include <QDebug>

class TDK_KinectV2Sensor : public TDK_Sensor
{
    Q_OBJECT
public:
    TDK_KinectV2Sensor();
    ~TDK_KinectV2Sensor();

    bool    mf_IsAvailable();
    bool    mf_SetupSensor();
    bool    mf_StartSensor();
    bool    mf_StopSensor();

    boost::function<void( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& )> mv_PointCloudCallback =
            [this]( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& ptr ){
        boost::mutex::scoped_lock lock(mv_Mutex);
        mf_SetMvPointCloud(ptr);

    };

protected:
    boost::mutex                            mv_Mutex;
    boost::shared_ptr<pcl::Kinect2Grabber>  mv_Grabber;
    boost::signals2::connection             mv_Connection;

public slots:
    void    mf_SlotUpdateFlagFilter();
    void    mf_SlotUpdateFilterBox();

};

#endif // TDK_KINECTV2SENSOR_H
