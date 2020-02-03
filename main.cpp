#include "mainwindow.h"
#include "kippi.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    std::string in;
    cv::CommandLineParser parser(argc, argv, "{@input|../samples/data/corridor.jpg|input image}{help h||show help message}");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    in = parser.get<string>("@input");

    Kippi k;
    k.partition(in);

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
