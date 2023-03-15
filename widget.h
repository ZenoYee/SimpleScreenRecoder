#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>

#include "cscreenrecorder.h"

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();

private:
    QVariantMap getUIArgs();

    void initUI();

    void initSlots();
private:
    Ui::Widget *ui;

    CScreenRecorder m_recoder;

private slots:
    void sltBtnClick();
};

#endif // WIDGET_H
