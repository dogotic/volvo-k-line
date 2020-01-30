#ifndef OBDDEVICEQUERYTHREAD_H
#define OBDDEVICEQUERYTHREAD_H

#include <QObject>
#include <QThread>
#include "obddevicequerythread.h"

class OBDDeviceQueryThread : public QThread
{
    Q_OBJECT

protected:
    void run();

public:
    OBDDeviceQueryThread(QStringList *portList);
    bool isDeviceFound() { return obdScannerFound; }

private:
    QStringList *queryThread_portNameList;
    bool obdScannerFound = false;

public slots:
    void OBDDevicePortQueryFinished();
};

#endif // OBDDEVICEQUERYTHREAD_H
