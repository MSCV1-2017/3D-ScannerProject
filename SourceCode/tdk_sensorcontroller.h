#ifndef TDK_SENSORCONTROLLER_H
#define TDK_SENSORCONTROLLER_H

//Include QT classes
#include <QObject>
#include <map>

//Include custom classes
#include "tdk_kinectv2sensor.h"
#include "tdk_intelr200sensor.h"

/******************************************************************************
 * Description       : Controller class to manage all the sensors
 * Author            : Software Unicorns
 *
 *****************************************************************************/
class TDK_SensorController : public QObject
{
    Q_OBJECT
public:
    //Constructor and Destructor
    explicit TDK_SensorController(QObject *parent = 0);
    ~TDK_SensorController();

    void                            mf_InitializeSensors            ();                     //Function to initialize all the sensors
    bool                            mf_IsSensorAvailable            ();                     //Function to check if any sensor is available
    std::map<QString, QString>      mf_GetAvailableSensorNames      ();                     //Function to get all the available sensor names
    TDK_Sensor                     *mf_GetSensor                    (QString sensorId);     //Function to get a sensor based on id

protected:
    std::map<QString, TDK_Sensor*>  mv_Sensors;                                             //Map to store all the sensors

};

#endif // TDK_SENSORCONTROLLER_H
