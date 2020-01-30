#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <QMessageBox>

#include "obddevicequerythread.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setComPortConnectionStatus(false);
    setECUConnectionStatus(false);

    const auto infos = QSerialPortInfo::availablePorts();

    if (infos.size() > 0)
    {
         portsFound = true;
         portNameList = new QStringList();

         /* iterrate over the list of ports */
         for (const QSerialPortInfo &info : infos)
         {
            portNameList->append(info.portName());
         }
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_Connect_clicked()
{
    if (portsFound)
    {
       queryThread = new OBDDeviceQueryThread(portNameList);
       connect(queryThread,SIGNAL(finished()),this,SLOT(OBDDevicePortQueryFinished()));

        /* start diagnostic device querry by creating a query thread */
        queryThread->start();
        /* disable button until the query is completed */
        ui->pushButton_Connect->setDisabled(true);
    }
    else
    {
        /* display messagebox and set COM PORT CONNECTION STATUS TO DISCONNECTED */
        setComPortConnectionStatus(false);
        setECUConnectionStatus(false);
        QMessageBox msgBox;
        msgBox.setText("NO OBD SCANNER FOUND");
        msgBox.exec();
    }
}

void MainWindow::OBDDevicePortQueryFinished()
{
    qDebug() << "Port Query Finished";
    if (queryThread->isDeviceFound())
    {
        /* set COM PORT CONNECTION STATUS TO CONNECTED */
        setComPortConnectionStatus(true);
    }
    else
    {
        /* display messagebox and set COM PORT CONNECTION STATUS TO DISCONNECTED */
        setComPortConnectionStatus(false);
        setECUConnectionStatus(false);
        QMessageBox msgBox;
        msgBox.setText("NO OBD SCANNER FOUND");
        msgBox.exec();
    }
    ui->pushButton_Connect->setEnabled(true);
    delete queryThread;
}


void MainWindow::setComPortConnectionStatus(bool isConnected)
{
    if (isConnected)
    {
        obdScannerConnected = true;

        QPalette sample_palette;
        sample_palette.setColor(QPalette::Window, Qt::darkGreen);
        sample_palette.setColor(QPalette::WindowText, Qt::white);

        ui->labelComPortConnectionStatus->setAutoFillBackground(true);
        ui->labelComPortConnectionStatus->setPalette(sample_palette);
        ui->labelComPortConnectionStatus->setText("CONNECTED");
    }
    else
    {
        obdScannerConnected = false;

        QPalette sample_palette;
        sample_palette.setColor(QPalette::Window, Qt::red);
        sample_palette.setColor(QPalette::WindowText, Qt::white);

        ui->labelComPortConnectionStatus->setAutoFillBackground(true);
        ui->labelComPortConnectionStatus->setPalette(sample_palette);
        ui->labelComPortConnectionStatus->setText("DISCONNECTED");
    }
}

void MainWindow::setECUConnectionStatus(bool isConnected)
{
    if (isConnected)
    {
        ecuConnected = true;

        QPalette sample_palette;
        sample_palette.setColor(QPalette::Window, Qt::darkGreen);
        sample_palette.setColor(QPalette::WindowText, Qt::white);

        ui->labelECUConnectionStatus->setAutoFillBackground(true);
        ui->labelECUConnectionStatus->setPalette(sample_palette);
        ui->labelECUConnectionStatus->setText("CONNECTED");
    }
    else
    {
        ecuConnected = false;

        QPalette sample_palette;
        sample_palette.setColor(QPalette::Window, Qt::red);
        sample_palette.setColor(QPalette::WindowText, Qt::white);

        ui->labelECUConnectionStatus->setAutoFillBackground(true);
        ui->labelECUConnectionStatus->setPalette(sample_palette);
        ui->labelECUConnectionStatus->setText("DISCONNECTED");
    }
}
