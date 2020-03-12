#include "mainwindow.h"

//Commenter si non Kippi
#include "kippi.h"
#include <QApplication>
#include <QDebug>
#include <utility>
#include <QVector2D>
//Commenter si non Kippi
#include <opencv2/core/types.hpp> //Point2f

int main(int argc, char *argv[])
{
    // DEBUT KIPPI, A COMMENTER SI VOUS N'AVEZ PAS OPENCV
    std::string in;
    cv::CommandLineParser parser(argc, argv, "{@input|../samples/data/corridor.jpg|input image}{help h||show help message}");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    in = parser.get<string>("@input");

    Kippi k;

    //Ajouter un second paramètre "true" à partition pour afficher les images de Kippi
    std::pair<std::vector<std::vector<QVector2D>>, std::vector<double>> p = k.partition(in, true);
    std::vector<std::vector<QVector2D>> contours = p.first;
    std::vector<double> medianValues = p.second;

    for(int i = 0 ; i < contours.size() ; ++i ){
        for(int j = 0 ; j < contours[i].size() ; ++j ){
            std::cout << "Contour " << i << ", Point " << j << ", X: " << contours[i][j].x() <<
                         ", Y: " << contours[i][j].y() << std::endl;
        }

        std::cout << "Contour " << i << ", Valeur médiane: " << medianValues[i] << std::endl << std::endl;
    }
    //FIN KIPPI

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
