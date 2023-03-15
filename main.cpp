#include "widget.h"
#include <QApplication>
#include <QProcess>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
//    QProcess::execute("ulimit -d 1000000000");
//    QProcess::execute("ulimit -s 1000000000");
    Widget w;
    w.show();

    return a.exec();
}
