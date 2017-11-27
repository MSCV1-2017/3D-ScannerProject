#ifndef TDK_INTELR200SENSOR_H
#define TDK_INTELR200SENSOR_H

//R200 libraries
#include <pxcsession.h>
#include <pxcsensemanager.h>
#include<pxcprojection.h>
#include<pxccapture.h>

//PCL libraries
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/boost.h>
#include "tdk_sensor.h"

class TDK_IntelR200Sensor : public TDK_Sensor
{
public:
    //default constructor
    TDK_IntelR200Sensor();
    //a constructor with parameters
    //copy constructor

    //default destructor
    ~TDK_IntelR200Sensor();

    //returns true if the r200 is available
    bool mf_IsAvailable();

    //sets up sensor so that a point cloud may be acquired (also initializes the manager interface)
    bool mf_SetupSensor();

    //starts the r200 sensor (initializes the manager interface)
    bool mf_StartSensor();

    //stops the r200 sensor (by releasing the manager interface)
     bool mf_StopSensor();

    //returns pointer to a point cloud of type pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    void mf_GeneratePointCloud();

    //thread runs in the background
    boost::thread mv_thread;

    //threadfunction acquires point cloud
    void mf_threadAcquireCloud();

protected:
    //point cloud container
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr mv_cloud;

    bool mv_Available;

    //color and depth images returned by the camera
    PXCImage *mv_colorImage, *mv_depthImage;

    //parameters for color and depth images to be acquired by the camera
    int mv_colorWidth, mv_colorHeight, mv_fps;
    int mv_depthWidth, mv_depthHeight;

    //aligned color and depth imagereturned by the camera
    PXCCapture::Sample *mv_alignedImage;

    //senseManager
    PXCSenseManager *mv_myManager;

    //sessionManager
    PXCSession *mv_session;

    //captureManager (for creating the projection interface)
    PXCCapture *mv_capture;

    //deviceManager
    PXCCapture::Device *mv_device ;

    //projection interface
    PXCProjection *mv_projection;

    //mv for quitting stream
    bool mv_quit;

    mutable boost::mutex mutex;

};

#endif // TDK_INTELR200SENSOR_H
