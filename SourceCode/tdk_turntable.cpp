#include "tdk_turntable.h"
#include <QCoreApplication>

QT_USE_NAMESPACE


TDK_Turntable::TDK_Turntable( QObject *parent)
    : QObject(parent)
    , mv_SerialPort(new QSerialPort)
    , mv_TotalAngle(0)
    , mv_TotalRotations(0)
    , mv_FlagRunning(false)
{

}

TDK_Turntable::~TDK_Turntable()
{

}

//function that sends via serial the command for starting the turntable
void TDK_Turntable::mf_StartPlatform(QString serialPortName, int serialBaudRate)
{
    qDebug() << "Starting platform";
    mv_SerialPort->setPortName(serialPortName);
    mv_SerialPort->setBaudRate(serialBaudRate);
    mv_SerialPort->open(QIODevice::ReadWrite);
    mv_FlagRunning = true;

    connect(mv_SerialPort, &QSerialPort::readyRead, this, &TDK_Turntable::mf_SlotHandleReadyRead);
    mv_TotalAngle = 0;
    this->mf_SendCommandViaSerial(START);
}


//function that sends via serial the command for stopping the turntable
void TDK_Turntable::mf_StopPlatform()
{
    disconnect(mv_SerialPort, &QSerialPort::readyRead, this, &TDK_Turntable::mf_SlotHandleReadyRead);
    mv_FlagRunning = false;
    this->mf_SendCommandViaSerial(STOP);
    if (mv_SerialPort->isOpen())
        mv_SerialPort->close();
}

bool TDK_Turntable::mf_IsRunning()
{
    return mv_FlagRunning;
}


//private function that sends a command via serial in an array
void TDK_Turntable::mf_SendCommandViaSerial(int command)
{
    qDebug() << "Command received " << command;
    if (mv_SerialPort->isOpen() && mv_SerialPort->isWritable())
    {
        qDebug() << "Command sent to arduino";
        QByteArray dayArray;
        dayArray[0]=command;
        mv_SerialPort->write(dayArray);
        mv_SerialPort->flush();
        bool ret = mv_SerialPort->waitForBytesWritten(2);
        if (!ret){
            qDebug("Could not write in the serial port. Try again!");
        }
    }
    else
    {
        qDebug("Couldn't write in the serial port. Try again!");
    }
}


//////////////////////////////////////////////////////////////////////////////////////////
//Slot function for when the turntable has rotated 5 degrees
void TDK_Turntable::mf_SlotHandleReadyRead()
{
    static int i=0;
    mv_ReadData.append(mv_SerialPort->readAll());
    mv_TotalAngle += (int)mv_ReadData[i];
    qDebug() << mv_TotalAngle;
    qDebug() << "Step angle is " << mv_StepAngle;

    //Signal is emitted every time we accomplish the step degrees
    if (mv_TotalAngle % mv_StepAngle == 0)
    {
        qDebug() << "-------------";
        emit (mf_SignalStepAngleRotated(mv_StepAngle));
    }

    //signal is emitted every when total rotation is accomplished
    if (mv_TotalAngle / 360 >= mv_TotalRotations)
    {
        mf_StopPlatform();
        emit (mf_SignalRotationsDone());
    }

    i++;

}

//////////////////////////////////////////////////////////////////////////////////////

int TDK_Turntable::mf_GetTotalRotations() const
{
    return mv_TotalRotations;
}

void TDK_Turntable::mf_SetTotalRotations(int value)
{
    mv_TotalRotations = value;
}

int TDK_Turntable::mf_GetStepAngle() const
{
    return mv_StepAngle;
}

void TDK_Turntable::mf_SetStepAngle(int value)
{
    mv_StepAngle = value;
}
