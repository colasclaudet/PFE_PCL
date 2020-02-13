#include "fragment.h"
#include <iostream>


Fragment::Fragment()
{
    type = Type::Inconnu;
}
Fragment::Type Fragment::getType(){
    return type;
}
std::string Fragment::getStringType(){
    switch (type) {
        case Type::Mur :
            return "Mur";
            break;
    case Type::Plafond :
        return "Plafond";
        break;
    case Type::Sol :
        return "Sol";
        break;
    default :
        return "Autre";
        break;
    }
}
QVector3D Fragment::getMeanPosition(){
    return meanPosition;
}
void Fragment::addPlane(pcl::PointCloud<pcl::PointXYZRGB> plane){
    planes.push_back(plane);    
}

void Fragment::setType(Type t){
    type = t;
}

void Fragment::computeMeanPosition(){
    QVector3D result(0,0,0);
    int cpt;
    for (unsigned i = 0; i < planes.size(); i++){
        for (size_t j=0;j< planes[i].points.size();j++){
            result[0] += planes[i].points[j].x;
            result[1] += planes[i].points[j].y;
            result[2] += planes[i].points[j].z;
            cpt++;
        }
    }

    result[0] /= cpt;
    result[1] /= cpt;
    result[2] /= cpt;
    meanPosition = result;
}

void Fragment::attributeColorAuto(){
    int r = 0;
    int g = 0;
    int b = 0;

    if (type == Type::Mur){
        b = 255;
    } else if (type == Type::Plafond){
        g = 255;
    } else if (type == Type::Sol){
        r = 255;
    } else {
        r = 255; g = 255; b = 255;
    }

    for (unsigned i = 0; i < planes.size(); i++){
        std::cout << "nb point dansplan : " << planes[i].points.size()<< std::endl;
        for (size_t j=0;j< planes[i].points.size();j++){
            planes[i].points[j].r = r;
            planes[i].points[j].g = g;
            planes[i].points[j].b = b;
        }
    }
    //TODO  coloriage marche aps
    std::cout << "couleur " << r << ", " << g << ", " << b << "attribué, type : " << getStringType() << std::endl;
}

