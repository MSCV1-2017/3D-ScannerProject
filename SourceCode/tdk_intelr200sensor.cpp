#include "tdk_intelr200sensor.h"
#include <QException>

//constructor
TDK_IntelR200Sensor::TDK_IntelR200Sensor():TDK_Sensor(), mv_Available(false)
{
    mf_SetMvId(QString("INTELR200"));
    mf_SetMvName(QString("Intel R200"));

    mf_SetupSensor();
    mv_quit = true;

}

//destructor
TDK_IntelR200Sensor::~TDK_IntelR200Sensor()
{
    mv_myManager->Close();
    mv_myManager->Release();
    mv_projection->Release();

}

//returns true if sensor is connected
bool TDK_IntelR200Sensor::mf_IsAvailable()
{
    //qDebug() << "sensor is connected = " << mv_myManager->IsConnected();
    //return ((bool) mv_myManager->IsConnected());
    return mv_Available;
}

//stops sensor
bool TDK_IntelR200Sensor::mf_StopSensor()
{
    //release following interfaces : manager, projection
    //also releases device and session

    mv_quit = true;

    return true;
}

//generates a pcl pointcloud from sensor streams
void TDK_IntelR200Sensor::mf_GeneratePointCloud()
{
    //Aligned capture: returns aligned color and depth streams: block/stop return until both streams are ready
    //return NULL pointer if error in acquiring frame
    if (mv_myManager->AcquireFrame(true)<PXC_STATUS_NO_ERROR) {
        //       qDebug() <<"Error acquiring aligned frames!";
        return;
    }

    // retrieve the color and depth samples aligned
    mv_alignedImage = mv_myManager->QuerySample();
    //qDebug() << mv_alignedImage->IsEmpty();

    // work on color and depth streams of
    mv_colorImage = mv_alignedImage->color;
    mv_depthImage = mv_alignedImage->depth;
    //    qDebug() << "color and depth streams extracted from alignedImage";

    //map the color image to depth image (Note: kinect grabber class maps depth to color)
    PXCImage* colorMappedToDepth;
    colorMappedToDepth = mv_projection->CreateColorImageMappedToDepth(mv_depthImage, mv_colorImage);

    // go fetching the next aligned-sample, if required : (this does not 'release' the manager interface)
    mv_myManager->ReleaseFrame();
    //  qDebug() << "mv_myManager released frame, can fetch next aligned frames";

    //-----For creating point cloud from depth image (by <projecting> depth image to world coordinates)-------

    //create an ImageData object to store info (in an ImageInfo object) about PXCImage depthImage
    PXCImage::ImageData depthImageData;
    PXCImage::ImageData colorMappedToDepthData;

    PXCImage::ImageInfo depthImageInfo = mv_depthImage->QueryInfo();
    PXCImage::ImageInfo colorMappedToDepthInfo = colorMappedToDepth->QueryInfo();

    //For checking storage formats of depth and color data buffers
    //depthImageData.format = depthImageInfo.format;
    colorMappedToDepthData.format = colorMappedToDepthInfo.format;

    //Initialize planes and pitches arrays, depthImage buffer data starts at ..data.planes[0]
    depthImageData.planes[0] = {0};
    depthImageData.pitches[0] = {0};

    colorMappedToDepthData.planes[0] = {0};
    colorMappedToDepthData.pitches[0] = {0};

    //Acquire read access to depthImage buffer: this is equivalent to getting buffer positions in memory
    //in depthImageData (so that we know where to read)
    mv_depthImage -> AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &depthImageData);
    colorMappedToDepth->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &colorMappedToDepthData);

    //short = 2 bytes: each depthImage pixel value is 16 bits long, so we want to traverse the
    //depth buffer in steps of 2 bytes; similarly, color buffer is in RGB32 format (8x4 -> R,G,B,A),
    //sizeof(uint8_t) = 1

    short *depthbuffer = (short*) depthImageData.planes[0];
    uint8_t *mappedColorbuffer = (uint8_t*) colorMappedToDepthData.planes[0];

    //    qDebug() << "Acquired buffer locations of depth and color streams";

    //point cloud container for the current request (session)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ( new pcl::PointCloud <pcl::PointXYZRGB> () );

    for (int row = 0; row < mv_depthHeight; row ++){
        for (int col = 0; col < mv_depthWidth; col++){
            PXCPoint3DF32 depthImagePoint;
            PXCPoint3DF32 worldPoint;
            pcl::PointXYZRGB pclworldpoint;

            depthImagePoint.x = col;
            depthImagePoint.y = row;
            depthImagePoint.z = depthbuffer[0];

            mv_projection->ProjectDepthToCamera(1, &depthImagePoint, &worldPoint);

            pclworldpoint.x = (float) worldPoint.x/1000.0;
            pclworldpoint.y = (float) worldPoint.y/1000.0;
            pclworldpoint.z = (float) -worldPoint.z/1000.0;

            pclworldpoint.b = (float) mappedColorbuffer[0];
            pclworldpoint.g = (float) mappedColorbuffer[1];
            pclworldpoint.r = (float) mappedColorbuffer[2];

            if (mv_FlagFilterPoints){
                if (pclworldpoint.x > mv_XMin & pclworldpoint.x < mv_XMax &
                        pclworldpoint.y > mv_YMin & pclworldpoint.y < mv_YMax &
                        pclworldpoint.z > mv_ZMin & pclworldpoint.z < mv_ZMax){

                    cloud->points.push_back(pclworldpoint);
                }
            }

            else
            {cloud->points.push_back(pclworldpoint);}

            depthbuffer++;
            mappedColorbuffer += 4;
        }
    }

    // mv_myManager->Release();
    //    qDebug() << "About to set point cloud";
    mf_SetMvPointCloud(cloud);

    //   qDebug() << "Point cloud set";

}

//keeps updating mv_cloud in the background
void TDK_IntelR200Sensor::mf_threadAcquireCloud()
{
    //pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
    //cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    //    qDebug() << "Intel R200 entering thread function";
    while (!mv_quit){
        boost::unique_lock<boost::mutex> lock(mutex);
        //        qDebug() << "Intel R200 inside thread while";

        mf_GeneratePointCloud();

        //qDebug() << "Intel R200 exiting thread function";
        lock.unlock();
    }
}


//set dimensions of color and depth streams and enable their captures, and initialize other member variables
bool TDK_IntelR200Sensor::mf_SetupSensor ()
{
    qDebug() << "IntelR200 Setup sensor";
    //Notes:
    //Use sensemanager for applications like hand tracking,
    //face tracking, etc., and for organizing and controlling multimodal
    //pipelines; controls = pipeline.{start, stop, pause, getFrame resume}
    //here: mv_myManager provides interface to image acquisition pipeline and the current session and device
    mv_myManager = PXCSenseManager::CreateInstance();
    qDebug() << "Manager instance created" ;
    //Notes:
    //The EnableStream[s] function requests that the specified
    //stream(s) be part of the streaming pipeline. The application
    //can call this function multiple times for different streams.
    //Infrared stream is not available in r200
    mv_colorWidth = 640, mv_colorHeight = 480, mv_fps = 30;
    mv_depthWidth = 320, mv_depthHeight = 240;

    mv_myManager->EnableStream(PXCCapture::STREAM_TYPE_COLOR, mv_colorWidth, mv_colorHeight, mv_fps);
    mv_myManager->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, mv_depthWidth, mv_depthHeight, mv_fps);

    qDebug() << "Streams enabled";

    pxcStatus initStatus = mv_myManager->Init();

    if(initStatus == 0){
        mv_Available = true;
    }
    else{
        mv_Available = false;
        return false;
    }
    qDebug() << "manager initialized status: " << initStatus;

    //Initialize session, capture and device interfaces
    mv_session = mv_myManager->QuerySession();

    qDebug() << "session created from manager";

    PXCSession::ImplDesc desc={};
    desc.group=PXCSession::IMPL_GROUP_SENSOR;
    desc.subgroup=PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;
    PXCSession::ImplDesc desc1;
    pxcStatus sts = mv_session->QueryImpl(&desc,0,&desc1);
    sts = mv_session->CreateImpl<PXCCapture>(&desc1,&mv_capture);


    qDebug() << "stuff done for creating device";

    mv_device = mv_capture->CreateDevice(0);

    qDebug() << "device created";

    //initialize the projection interface (for mapping between depth, color and world (camera) coordinates)



    qDebug()<< "connected = "<< mv_myManager->IsConnected();
    mv_projection = mv_device->CreateProjection();

    qDebug() << "try block end";


    qDebug() << "IntelR200 Setup done";
    mv_Available = true;
    return true;
}

bool TDK_IntelR200Sensor::mf_StartSensor()
{
    qDebug() << "Starting Intel R200";
    mv_quit = false;


    mv_thread = boost::thread(&TDK_IntelR200Sensor::mf_threadAcquireCloud, this);
    return true;
}
