#ifndef TDK_TURNTABLE_H
#define TDK_TURNTABLE_H

#include <QtSerialPort/QSerialPort>
#include <QTextStream>
#include <QTimer>
#include <QByteArray>
#include <QObject>
#include <QDebug>

/*
QT_USE_NAMESPACE
QT_BEGIN_NAMESPACE
QT_END_NAMESPACE
*/

class TDK_Turntable : public QObject
{
    Q_OBJECT

public:
    TDK_Turntable(QObject *parent = 0);
    ~TDK_Turntable();
    void mf_StartPlatform(QString serialPortName, int serialBaudRate);
    void mf_StopPlatform();
    bool mf_IsRunning();

    int mf_GetStepAngle() const;
    void mf_SetStepAngle(int value);

    int mf_GetTotalRotations() const;
    void mf_SetTotalRotations(int value);

signals:
    void mf_SignalRotationsDone();
    void mf_SignalStepAngleRotated(int currentAngle);

private slots:
    void mf_SlotHandleReadyRead();

private:
    void mf_SendCommandViaSerial(int command);
    QSerialPort *mv_SerialPort;
    QByteArray  mv_ReadData;

    int         mv_TotalAngle;
    int         mv_StepAngle;
    int         mv_TotalRotations;
    bool        mv_FlagRunning;

    enum Command{START = 1, STOP = 0};

};


#endif // TDK_TURNTABLE_H
