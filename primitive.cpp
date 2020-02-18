#include "primitive.h"

using namespace std;
using namespace cv;

adjacency_list<listS, listS, undirectedS, GraphVertex>::vertex_descriptor Primitive::lastIntersection() const
{
    return m_lastIntersection;
}

void Primitive::setLastIntersection(const adjacency_list<listS, listS, undirectedS, GraphVertex>::vertex_descriptor &value)
{
    m_lastIntersection = value;
}

Primitive::Primitive(Point2f origin, Point2f end, int size, int K, int idPrimitive, int idSegment) : m_origin(origin), m_end(end), m_current(end),
    m_K(K), m_idPrimitive(idPrimitive), m_idSegment(idSegment)
{
    m_certificates = vector<bool>(size, 1);
    m_certificates[m_idPrimitive] = 0;

    m_direction = m_end - m_origin;
    double norm = sqrt(m_direction.x * m_direction.x + m_direction.y * m_direction.y);
    m_direction.x /= norm;
    m_direction.y /= norm;
}

Point2f Primitive::origin() const
{
    return m_origin;
}

Point2f Primitive::end() const
{
    return m_end;
}

Point2f Primitive::current() const
{
    return m_current;
}

void Primitive::setCurrent(const Point2f &current)
{
    m_current = current;
}

Point2f Primitive::direction() const
{
    return m_direction;
}

int Primitive::idSegment() const
{
    return m_idSegment;
}

int Primitive::idPrimitive() const
{
    return m_idPrimitive;
}

vector<bool> Primitive::certificates()
{
    return m_certificates;
}

int Primitive::K() const
{
    return m_K;
}

void Primitive::setK(int K)
{
    m_K = K;
}

int Primitive::nbCertificates() const
{
    int nb = 0;
    for(int i = 0; i < m_certificates.size(); ++i){
        if(m_certificates[i] == 0) ++nb;
    }

    return nb;
}

void Primitive::flipCertificate(int index){
    m_certificates[index] = 1 - m_certificates[index];
}

bool Primitive::operator <(const Primitive & p) const
{
    return nbCertificates() < p.nbCertificates();
}
