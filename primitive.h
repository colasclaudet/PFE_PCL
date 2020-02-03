#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include <cmath>
#include <vector>
#include <opencv2/core/types.hpp>

using namespace std;
using namespace cv;

class Primitive
{
private:
    Point2f m_origin;
    Point2f m_end;
    Point2f m_current;
    Point2f m_direction;
    //id du segment dans lines_fld
    int m_idSegment;
    //id de la primitve en elle-même
    int m_idPrimitive;
    vector<bool> m_certificates;
    int m_K;
public:
    Primitive(Point2f origin, Point2f end, int size, int K, int idPrimitive, int idSegment);
    int nbCertificates() const;
    bool operator <(const Primitive & p) const;

    Point2f origin() const;

    Point2f end() const;

    Point2f current() const;
    void setCurrent(const Point2f &current);

    Point2f direction() const;

    int idSegment() const;

    int idPrimitive() const;

    vector<bool> certificates();

    int K() const;
    void setK(int K);
};

#endif // PRIMITIVE_H
