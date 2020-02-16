#ifndef VERTEX_H
#define VERTEX_H

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
    GraphVertex();
    GraphVertex(const GraphVertex &g);

    int i() const;

    int j() const;

    Point2f intersection() const;
};

#endif // VERTEX_H
