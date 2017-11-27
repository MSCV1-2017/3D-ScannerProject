#include "tdk_sensorcontroller.h"

/***************************************************************************
 * Input argument(s) : QObject *parent - Parent class pointer
 * Return type       : NA
 * Functionality     : Constructor to initialize variables
 *
 **************************************************************************/
TDK_SensorController::TDK_SensorController(QObject *parent) : QObject(parent)
{

}

/***************************************************************************
 * Input argument(s) : NA
 * Return type       : NA
 * Functionality     : Destructor to free variables
 *
 **************************************************************************/
TDK_SensorController::~TDK_SensorController()
{

}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : void
 * Functionality     : Function to initialize all the sensors and store
 *                     it in the sensors map.
 *
 **************************************************************************/
void TDK_SensorController::mf_InitializeSensors()
{
    //Initialize each sensor and store it in the sensor map
    TDK_Sensor *sensor = new TDK_KinectV2Sensor();
    mv_Sensors[sensor->mf_GetMvName()] = sensor;
    sensor = new TDK_IntelR200Sensor();
    mv_Sensors[sensor->mf_GetMvName()] = sensor;
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : bool - sensor availability flag
 * Functionality     : Function to check if any sensor is available and
 *                     returns a boolean variable based on availability.
 *
 **************************************************************************/
bool TDK_SensorController::mf_IsSensorAvailable()
{
    std::map<QString, TDK_Sensor*>::iterator it = mv_Sensors.begin();
    //Iterate over all the sensors to check the availability
    while(it != mv_Sensors.end()){
        if(((TDK_Sensor *)it->second)->mf_IsAvailable()){
            return true;
        }
        it++;
    }
    return false;
}

/***************************************************************************
 * Input argument(s) : void
 * Return type       : std::map<QString, QString> - sensor id and name map
 * Functionality     : Function to get names of all the sensors available.
 *
 **************************************************************************/
std::map<QString, QString> TDK_SensorController::mf_GetAvailableSensorNames()
{
    std::map<QString, TDK_Sensor*>::iterator it = mv_Sensors.begin();
    std::map<QString, QString> sensorNames;
    //Iterate over the sensor map to get the available sensor id and names
    while(it != mv_Sensors.end()){
        if( ((TDK_Sensor *)it->second)->mf_IsAvailable() ){
            sensorNames[it->first] = ((TDK_Sensor *)it->second)->mf_GetMvName();
        }
        it++;    
    }
    return sensorNames;
}

/***************************************************************************
 * Input argument(s) : QString sensorId - Id of requested sensor
 * Return type       : TDK_Sensor* - Pointer to requested sensor
 * Functionality     : Function to get a sensor given the sensor id.
 *
 **************************************************************************/
TDK_Sensor *TDK_SensorController::mf_GetSensor(QString sensorId)
{
    return mv_Sensors[sensorId];
}
