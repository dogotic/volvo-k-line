#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "obddevicequerythread.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void setComPortConnectionStatus(bool isConnected);
    void setECUConnectionStatus(bool isConnected);

private slots:
    void on_pushButton_Connect_clicked();

public slots:
    void OBDDevicePortQueryFinished();

private:
    Ui::MainWindow *ui;
    QStringList *portNameList;
    OBDDeviceQueryThread *queryThread;

    bool portsFound = false;
    bool obdScannerConnected= false;
    bool ecuConnected = false;

};

#endif // MAINWINDOW_H
