#include "graphvertex.h"

using namespace std;
using namespace cv;

GraphVertex::GraphVertex(int i, int j, Point2f intersection) : m_i(i), m_j(j), m_intersection(intersection) {}

GraphVertex::GraphVertex() {}

GraphVertex::GraphVertex(const GraphVertex &g) : m_i(g.i()), m_j(g.j()), m_intersection(g.intersection()) {}

int GraphVertex::j() const
{
    return m_j;
}

Point2f GraphVertex::intersection() const
{
    return m_intersection;
}

int GraphVertex::i() const
{
    return m_i;
}
