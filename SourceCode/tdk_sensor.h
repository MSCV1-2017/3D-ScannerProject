#ifndef TDK_SENSOR_H
#define TDK_SENSOR_H

//Include QT libraries
#include <QObject>
#include <QString>
#include <QDebug>
#include <map>

//Include PCL libraries
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/******************************************************************************
 * Description       : Interface class to be implemented by all sensor classes
 * Author            : Software Unicorns
 *
 *****************************************************************************/

class TDK_Sensor : public QObject
{
    Q_OBJECT
public:
    //Constructor and Destructor
    explicit TDK_Sensor(QObject *parent = 0);
    ~TDK_Sensor();

    //Pure virtual functions to be implemented in inherited classes
    virtual bool    mf_IsAvailable      () = 0;
    virtual bool    mf_SetupSensor      () = 0;
    virtual bool    mf_StartSensor      () = 0;
    virtual bool    mf_StopSensor       () = 0;

    //Function to set limits of filter box
    void    mf_SetFilterBox             (float xmin, float xmax, float ymin, float ymax, float zmin, float zmax);


    //Setter functions
    void    mf_SetMvId                  (const QString &value)                             {    mv_Id = value;              }
    void    mf_SetMvName                (const QString &value)                             {    mv_Name = value;            }
    void    mf_SetMvSensorDetails       (const std::map<QString, QString> &value)          {    mv_SensorDetails = value;   }
    void    mf_SetMvFlagFilterPoints    (bool value);
    void    mf_SetMvPointCloud          (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr
                                         &pointCloudPtr);

    //Getter functions
    QString                                     mf_GetMvId                  () const       {    return mv_Id;               }
    QString                                     mf_GetMvName                () const       {    return mv_Name;             }
    std::map<QString, QString>                  mf_GetMvSensorDetails       () const       {    return mv_SensorDetails;    }
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr mf_GetMvPointCloud          () const       {    return mv_PointCloud;       }
    bool                                        mf_GetMvFlagFilterPoints    () const       {    return mv_FlagFilterPoints; }

protected:
    QString     mv_Id;                                                      //Sensor id
    QString     mv_Name;                                                    //Sensor name

    bool        mv_FlagFilterPoints;                                        //Enable/Disable flag for filter box
    float       mv_XMin, mv_XMax;                                           //Filter min and max x values
    float       mv_YMin, mv_YMax;                                           //Filter min and max y values
    float       mv_ZMin, mv_ZMax;                                           //Filter min and max z values

    std::map<QString, QString>                      mv_SensorDetails;       //Map to store additional sensor details
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr     mv_PointCloud;          //Pointer to current point cloud captured by sensor

signals:
    void    mf_SignalPointCloudUpdated  ();                                 //Signals pointcloud update
    void    mf_SignalFlagFilterUpdated  ();                                 //Signals filter box flag update
    void    mf_SignalFilterBoxUpdated   ();                                 //Signals filter box update

};

#endif // TDK_SENSOR_H
