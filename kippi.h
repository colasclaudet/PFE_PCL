#ifndef KIPPI_H
#define KIPPI_H

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/ximgproc.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <queue>
#include "primitive.h"
#include "graphvertex.h"

using namespace std;
using namespace cv;
using namespace cv::ximgproc;
using namespace boost;

#define EPSILON 1E-10



class Kippi
{
private:
    typedef adjacency_list<listS, listS, undirectedS, GraphVertex> GRAPH;
    vector<Primitive> m_primitives;
    vector<vector<Point2f>> m_finalLines;

    Point2f middleOfSegment(const Point2f &segmentStart, const Point2f &segmentEnd);

    //Source: https://github.com/opencv/opencv/blob/master/modules/imgproc/src/min_enclosing_triangle.cpp#L1521-L1531
    bool almostEqual(double number1, double number2);
    double distanceBetweenPoints(const Point2f &a, const Point2f &b);
    bool arePointsColinear(const Point2f &a, const Point2f &b, const Point2f &c);
    bool pointOnSegment(const Point2f &segmentStart, const Point2f &segmentEnd, const Point2f &toEvaluate);

    //Source: https://github.com/opencv/opencv/blob/master/modules/imgproc/src/min_enclosing_triangle.cpp#L1380-L1424
    bool lineIntersection(const Point2f &a1, const Point2f &b1, const Point2f &a2,
                            const Point2f &b2, Point2f &intersection);

    bool isPropagationLeft();
    void propagation(double t, const Mat &image);
    void propagationPriorityQueue(double t, const Mat &image);

public:
    Kippi();
    vector<Primitive> primitives();
    GRAPH m_g;
    void partition(string imgName);
    vector<vector<Point2f> > finalLines();
};

#endif // KIPPI_H
