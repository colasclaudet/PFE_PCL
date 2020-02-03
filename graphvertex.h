#ifndef GRAPHVERTEX_H
#define GRAPHVERTEX_H

#include <opencv2/core/types.hpp>

using namespace std;
using namespace cv;

class GraphVertex
{

private:
    int m_i;
    int m_j;
    Point2f m_intersection;
public:
    GraphVertex(int i, int j, Point2f intersection);

    int i() const;

    int j() const;

    Point2f intersection() const;
};

#endif // VERTEX_H
