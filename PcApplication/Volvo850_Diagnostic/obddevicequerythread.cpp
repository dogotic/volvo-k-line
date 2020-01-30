#include "obddevicequerythread.h"

#include <QDebug>
#include <QSerialPort>
#include <QByteArray>

const char *request =  "WHOSITHERE?\r";
const char *wanted_response = "VOLVO850HERE";

OBDDeviceQueryThread::OBDDeviceQueryThread(QStringList *portList)
{
    queryThread_portNameList = portList;
}

void OBDDeviceQueryThread::run()
{
    for (int i=0; i<queryThread_portNameList->size(); i++)
    {
        QSerialPort serialPort;
        QByteArray ba;
        ba[0] = '\r';
        serialPort.setBaudRate(QSerialPort::BaudRate::Baud115200);
        serialPort.setParity(QSerialPort::Parity::NoParity);
        serialPort.setDataBits(QSerialPort::DataBits::Data8);
        serialPort.setStopBits(QSerialPort::StopBits::OneStop);
        serialPort.setPortName(queryThread_portNameList->at(i));
        serialPort.open(QSerialPort::QIODevice::ReadWrite);
        serialPort.flush();
        serialPort.clear();
        serialPort.write(ba);
        serialPort.waitForBytesWritten();
        serialPort.flush();
        serialPort.clear();
        serialPort.waitForReadyRead();
        QByteArray response = serialPort.readLine();


        qDebug() << response;
        serialPort.close();

        if (response == wanted_response)
        {
            obdScannerFound = true;
            break;
        }
        else
        {
            qDebug() << "Trying next port";
        }
    }
}
