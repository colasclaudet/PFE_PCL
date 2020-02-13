#ifndef FRAGMENT_H
#define FRAGMENT_H
#include <QVector3D>
#include <QDebug>

//PCL
#include <pcl/point_types.h>
#include <pcl/common/common.h>


class Fragment
{
public:
    enum Type { Mur, Plafond, Sol, Autre, A_Calculer, Inconnu };
    Fragment();
    void addPlane(pcl::PointCloud<pcl::PointXYZRGB>);
    void setType(Type);
    void computeMeanPosition();
    QVector3D getMeanPosition();
    void attributeColorAuto();
    Type getType();

    std::string getStringType();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> planes;
private :
    Type type;
    QVector3D meanPosition = QVector3D(0,0,0);


};

#endif // FRAGMENT_H
