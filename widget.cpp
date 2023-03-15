#include "widget.h"
#include "ui_widget.h"
#include <QAudioDeviceInfo>
#include <QScreen>

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    initUI();
    initSlots();
}

Widget::~Widget()
{
    delete ui;
}

QVariantMap Widget::getUIArgs()
{
    QVariantMap args;
    QScreen *screen=QGuiApplication::primaryScreen ();
    QRect rect=screen->availableGeometry();

    if(ui->cbbFileType->currentIndex() == 0)
        args["filePath"] = "screen.mp4";
    else
        args["filePath"] = "rtmp://127.0.0.1:1935/live/test001";

    args["width"] = rect.width();
    args["height"] = rect.height();

    args["fps"] = 30;

    args["audioBitrate"] = 128000;
    args["sampleRate"] = 44100;
    args["MicrophoneName"] = "audio=" + ui->cbbMicrophone->currentText();
    return args;
}

void Widget::initUI()
{
    ui->btnStart->setEnabled(true);
    ui->btnPause->setEnabled(false);
    ui->btnStop->setEnabled(false);

    QList<QAudioDeviceInfo> devList = QAudioDeviceInfo::availableDevices(QAudio::AudioInput);
    for(const auto& it : devList)
    {
        if(ui->cbbMicrophone->findText(it.deviceName()) < 0)
            ui->cbbMicrophone->addItem(it.deviceName());
    }

    ui->cbbFileType->addItem(QStringLiteral("保存到本地").toUtf8().data());
    ui->cbbFileType->addItem(QStringLiteral("直播推流").toUtf8().data());
}

void Widget::initSlots()
{
    connect(ui->btnStart, &QPushButton::clicked, this, &Widget::sltBtnClick);
    connect(ui->btnPause, &QPushButton::clicked, this, &Widget::sltBtnClick);
    connect(ui->btnStop, &QPushButton::clicked, this, &Widget::sltBtnClick);
}

void Widget::sltBtnClick()
{
    static bool bNeedInit = true;
    if(QObject::sender() == ui->btnStart)
    {
        ui->btnStart->setEnabled(false);
        ui->btnPause->setEnabled(true);
        ui->btnStop->setEnabled(true);
        if(bNeedInit)
        {
            m_recoder.Init(getUIArgs());
            bNeedInit = false;
        }
        m_recoder.Start();
    }
    else if(QObject::sender() == ui->btnPause)
    {
        ui->btnStart->setEnabled(true);
        ui->btnPause->setEnabled(false);
        ui->btnStop->setEnabled(true);
        m_recoder.Pause();
    }
    else if(QObject::sender() == ui->btnStop)
    {
        ui->btnStart->setEnabled(true);
        ui->btnPause->setEnabled(false);
        ui->btnStop->setEnabled(false);
        bNeedInit = true;
        m_recoder.Stop();
    }
}

