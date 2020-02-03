#include "kippi.h"


Kippi::Kippi()
{
    m_primitives = vector<Primitive>();
}

vector<Primitive> Kippi::primitives()
{
    return m_primitives;
}


Point2f Kippi::middleOfSegment(const Point2f &segmentStart, const Point2f &segmentEnd){
    Point2f middle;

    middle.x = (segmentStart.x + segmentEnd.x) / 2;
    middle.y = (segmentStart.y + segmentEnd.y) / 2;

    return middle;
}

//Source: https://github.com/opencv/opencv/blob/master/modules/imgproc/src/min_enclosing_triangle.cpp#L1521-L1531
bool Kippi::almostEqual(double number1, double number2){
    return (std::abs(number1 - number2) <= (EPSILON * max(max(1.0, std::abs(number1)), std::abs(number2))));
}

double Kippi::distanceBetweenPoints(const Point2f &a, const Point2f &b){
    double xDiff = b.x - a.x;
    double yDiff = b.y - a.y;

    return sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

bool Kippi::arePointsColinear(const Point2f &a, const Point2f &b, const Point2f &c){
    double crossproduct = (c.y - a.y) * (b.x - a.x) - (c.x - a.x) * (b.y - a.y);
    if(abs(crossproduct) > EPSILON) return false;
    return true;
}

bool Kippi::pointOnSegment(const Point2f &segmentStart, const Point2f &segmentEnd, const Point2f &toEvaluate){
    double dStart = distanceBetweenPoints(segmentStart, toEvaluate);
    double dEnd = distanceBetweenPoints(toEvaluate, segmentEnd);
    double segmentLength = distanceBetweenPoints(segmentStart, segmentEnd);

    return (almostEqual(dStart + dEnd, segmentLength));
}

//Source: https://github.com/opencv/opencv/blob/master/modules/imgproc/src/min_enclosing_triangle.cpp#L1380-L1424
bool Kippi::lineIntersection(const Point2f &a1, const Point2f &b1, const Point2f &a2,
                             const Point2f &b2, Point2f &intersection){
    double A1 = b1.y - a1.y;
    double B1 = a1.x - b1.x;
    double C1 = (a1.x * A1) + (a1.y * B1);

    double A2 = b2.y - a2.y;
    double B2 = a2.x - b2.x;
    double C2 = (a2.x * A2) + (a2.y * B2);

    double det = (A1 * B2) - (A2 * B1);

    if (!almostEqual(det, 0)) {
        intersection.x = static_cast<float>(((C1 * B2) - (C2 * B1)) / (det));
        intersection.y = static_cast<float>(((C2 * A1) - (C1 * A2)) / (det));

        //std::cout << "X: " << intersection.x << std::endl;
        //std::cout << "Y: " << intersection.y << std::endl;

        if(pointOnSegment(a1, b1, intersection) && pointOnSegment(a2, b2, intersection))
            return true;
    }

    return false;
}

bool Kippi::isPropagationLeft(){
    for(int i = 0; i < m_primitives.size(); ++i){
        if(m_primitives[i].K() != 0) return true;
    }
    return false;
}

void Kippi::propagation(double t, const Mat &image){
    for(int i = 0; i < m_primitives.size(); ++i){
        if(m_primitives[i].K() == 0) continue;
        Point2f end(m_primitives[i].end());
        Point2f direction = m_primitives[i].direction();
        end.x += direction.x * t;
        end.y += direction.y * t;
        m_primitives[i].setCurrent(end);
        if(m_primitives[i].current().x < 0 ||m_primitives[i].current().y < 0 ||
                m_primitives[i].current().x > image.cols || m_primitives[i].current().y > image.rows) m_primitives[i].setK(0);
    }

    for(int i = 0; i < m_primitives.size(); ++i){
        if(m_primitives[i].K() == 0) continue;
        for(int j = 0; j < m_primitives.size(); ++j){
            if(i == j) continue;
            if(m_primitives[i].idSegment() == m_primitives[j].idSegment()) continue;
            Point2f current = m_primitives[i].current();
            Point2f origin = m_primitives[j].origin();
            Point2f end = m_primitives[j].current();

            if(!arePointsColinear(origin, end, current)) continue;
            if(!pointOnSegment(origin, end, current)) continue;

            m_primitives[i].certificates().at(j) = 0;

            GraphVertex v(i, j, current);
            add_vertex(v, m_g);

            m_primitives[i].setK(m_primitives[i].K() - 1);
        }
    }
}

void Kippi::partition(string imgName){
    Mat image = imread(imgName, IMREAD_GRAYSCALE);
    if( image.empty() )
    {
        exit(0);
    }

    Mat image_fld(image);

    // Create FLD detector
    // Param               Default value   Description
    // length_threshold    10            - Segments shorter than this will be discarded
    // distance_threshold  1.41421356    - A point placed from a hypothesis line
    //                                     segment farther than this will be
    //                                     regarded as an outlier
    // canny_th1           50            - First threshold for
    //                                     hysteresis procedure in Canny()
    // canny_th2           50            - Second threshold for
    //                                     hysteresis procedure in Canny()
    // canny_aperture_size 3             - Aperturesize for the sobel
    //                                     operator in Canny()
    // do_merge            false         - If true, incremental merging of segments
    //                                     will be perfomred
    int length_threshold = 20;
    float distance_threshold = 1.41421356f;
    double canny_th1 = 50.0;
    double canny_th2 = 50.0;
    int canny_aperture_size = 3;
    bool do_merge = true;
    Ptr<FastLineDetector> fld = createFastLineDetector(length_threshold,
            distance_threshold, canny_th1, canny_th2, canny_aperture_size,
            do_merge);
    vector<Vec4f> lines_fld;
    fld->detect(image, lines_fld);

    map<pair<int, int>, Point2f> intersections;

    GraphVertex cornerUpperLeft(-1, -1, Point2f(0, 0));
    GraphVertex cornerUpperRight(-2, -2, Point2f(0, image.cols));
    GraphVertex cornerLowerRight(-3, -3, Point2f(image.rows, image.cols));
    GraphVertex cornerLowerLeft(-4, -4, Point2f(image.rows, 0));

    add_vertex(cornerUpperLeft, m_g);
    add_vertex(cornerUpperRight, m_g);
    add_vertex(cornerLowerRight, m_g);
    add_vertex(cornerLowerLeft, m_g);

    int idPrimitive = 0;
    for(int i = 0 ; i < lines_fld.size() ; ++i){
        Point2f origin0(lines_fld[i][0], lines_fld[i][1]);
        Point2f end0(lines_fld[i][2], lines_fld[i][3]);
        Point2f middle = middleOfSegment(origin0, end0);
        GraphVertex v(i, i, middle);
        Primitive p0(middle, origin0, lines_fld.size() * 2, 1, idPrimitive, i);
        ++idPrimitive;
        Primitive p1(middle, end0, lines_fld.size() * 2, 1, idPrimitive, i);
        ++idPrimitive;
        m_primitives.push_back(p0);
        m_primitives.push_back(p1);
        add_vertex(v, m_g);
    }

    for(int i = 0 ; i < m_primitives.size() ; ++i){
        for(int j = i + 1 ; j < m_primitives.size() ; ++j){
            //if(primitives[i].m_idSegment == primitives[j].m_idSegment) continue;
            Point2f intersection;
            if(lineIntersection(m_primitives[i].origin(), m_primitives[i].end(), m_primitives[j].origin(), m_primitives[j].end(), intersection)){
                pair<pair<int, int>, Point2f> p(make_pair(i, j), intersection);
                intersections.insert(p);
                m_primitives[i].certificates().at(j) = 0;
                m_primitives[j].certificates().at(i) = 0;
            }
        }
    }

    for (std::map<pair<int,int>, Point2f>::iterator it=intersections.begin(); it!=intersections.end(); ++it){
        GraphVertex v(it->first.first, it->first.second, it->second);
        add_vertex(v, m_g);
    }

    double t = 0;
    double delta = 0.1;
    while(isPropagationLeft()){
        /*priority_queue<PRIMITIVE> primitivesPriorityQueue;
        for(int i = 0 ; i < primitives.size() ; ++i){
            if(primitives[i].m_K > 0)
                primitivesPriorityQueue.push(primitives[i]);
        }*/
        propagation(t, image);
        //propagationPriorityQueue(t, primitivesPriorityQueue, primitives, g, line_image_fld);
        t += delta;
    }

    pair<GRAPH::vertex_iterator, GRAPH::vertex_iterator> v_it = vertices(m_g);
    for(; v_it.first != v_it.second; ++v_it.first){
        GRAPH::vertex_descriptor v = *v_it.first;
        std::cout << m_g[v].i() << " " << m_g[v].j() << " " << m_g[v].intersection().x << " " << m_g[v].intersection().y << endl;
    }

    for(int i = 0 ; i < m_primitives.size() ; ++i){
        Point2f origin(m_primitives[i].origin());
        Point2f end(m_primitives[i].current());
        line(image_fld, origin, end, CV_RGB(255, 0, 0), 1, LINE_AA);
    }

    imshow("FLD result", image_fld);
    waitKey();
}
