#include "kippi.h"


Kippi::Kippi()
{
    m_primitives = vector<Primitive>();
}

vector<Primitive> Kippi::primitives()
{
    return m_primitives;
}


vector<vector<Point2f> > Kippi::finalLines()
{
    return m_finalLines;
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
    double xDiff = a.x - b.x;
    double yDiff = a.y - b.y;

    return sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

bool Kippi::arePointsColinear(const Point2f &a, const Point2f &b, const Point2f &c){
    double crossproduct = (c.y - a.y) * (b.x - a.x) - (c.x - a.x) * (b.y - a.y);
    if(abs(crossproduct) > EPSILON) return false;
    return true;
}

bool Kippi::pointOnSegment(const Point2f &segmentStart, const Point2f &segmentEnd, const Point2f &toEvaluate){
    double dStart = distanceBetweenPoints(toEvaluate, segmentStart);
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
        intersection.x = ((C1 * B2) - (C2 * B1)) / (det);
        intersection.y = ((C2 * A1) - (C1 * A2)) / (det);

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
    for(int i = 4; i < m_primitives.size(); ++i){
        if(m_primitives[i].K() == 0) continue;
        Point2f end(m_primitives[i].end());
        Point2f direction = m_primitives[i].direction();
        end.x += direction.x * t;
        end.y += direction.y * t;
        m_primitives[i].setCurrent(end);
        /*if(m_primitives[i].current().x < 2 || m_primitives[i].current().y < 2 ||
                m_primitives[i].current().x >= image.cols - 3 || m_primitives[i].current().y >= image.rows - 3) m_primitives[i].setK(0);*/
        if(m_primitives[i].current().x <= 2){
            m_primitives[i].setK(0);
            Point2f newCurrent(m_primitives[i].current());
            newCurrent.x = 2;
            m_primitives[i].setCurrent(newCurrent);
            GraphVertex v(i, 3, m_primitives[i].current());
            add_vertex(v, m_g);
        }

        if(m_primitives[i].current().y <= 2){
            m_primitives[i].setK(0);
            Point2f newCurrent(m_primitives[i].current());
            newCurrent.y = 2;
            m_primitives[i].setCurrent(newCurrent);
            GraphVertex v(i, 0, m_primitives[i].current());
            add_vertex(v, m_g);
        }

        if(m_primitives[i].current().x >= image.cols - 3){
            m_primitives[i].setK(0);
            Point2f newCurrent(m_primitives[i].current());
            newCurrent.x = image.cols - 3;
            m_primitives[i].setCurrent(newCurrent);
            GraphVertex v(i, 1, m_primitives[i].current());
            add_vertex(v, m_g);
        }

        if(m_primitives[i].current().y >= image.rows - 3){
            m_primitives[i].setK(0);
            Point2f newCurrent(m_primitives[i].current());
            newCurrent.y = image.rows - 3;
            m_primitives[i].setCurrent(newCurrent);
            GraphVertex v(i, 2, m_primitives[i].current());
            add_vertex(v, m_g);
        }
    }

    for(int i = 4; i < m_primitives.size(); ++i){
        if(m_primitives[i].K() == 0) continue;
        for(int j = 4; j < m_primitives.size(); ++j){
            if(i == j) continue;
            if(m_primitives[i].certificates().at(j) == 0) continue;
            if(m_primitives[j].certificates().at(i) == 0) continue;
            if(m_primitives[i].idSegment() == m_primitives[j].idSegment()) continue;

            Point2f current = m_primitives[i].current();
            Point2f origin = m_primitives[j].origin();
            Point2f end = m_primitives[j].current();

            //if(arePointsColinear(m_primitives[j].origin(), m_primitives[j].end(), m_primitives[i].end())) continue;


            Point2f intersection;
            if(lineIntersection(m_primitives[i].origin(), m_primitives[i].current(), m_primitives[j].origin(), m_primitives[j].current(), intersection)){

                m_primitives[i].flipCertificate(j);
                m_primitives[j].flipCertificate(i);
                m_primitives[i].setCurrent(intersection);
                GraphVertex v(i, j, intersection);
                add_vertex(v, m_g);
                m_primitives[i].setK(m_primitives[i].K() - 1);
            }

        }
    }
}

void Kippi::propagationPriorityQueue(double t, const Mat &image){
    for(int i = 4; i < m_primitives.size(); ++i){
        if(m_primitives[i].K() == 0) continue;
        Point2f end(m_primitives[i].end());
        Point2f direction = m_primitives[i].direction();
        end.x += direction.x * t;
        end.y += direction.y * t;
        m_primitives[i].setCurrent(end);
        /*if(m_primitives[i].current().x < 0 || m_primitives[i].current().y < 0 ||
                m_primitives[i].current().x >= image.cols || m_primitives[i].current().y >= image.rows) m_primitives[i].setK(0);*/

        if(m_primitives[i].current().x <= 2){
            m_primitives[i].setK(0);
            Point2f newCurrent(m_primitives[i].current());
            newCurrent.x = 2;
            m_primitives[i].setCurrent(newCurrent);
            GraphVertex v(i, 3, m_primitives[i].current());
            add_vertex(v, m_g);
        }

        if(m_primitives[i].current().y <= 2){
            m_primitives[i].setK(0);
            Point2f newCurrent(m_primitives[i].current());
            newCurrent.y = 2;
            m_primitives[i].setCurrent(newCurrent);
            GraphVertex v(i, 0, m_primitives[i].current());
            add_vertex(v, m_g);
        }

        if(m_primitives[i].current().x >= image.cols - 3){
            m_primitives[i].setK(0);
            Point2f newCurrent(m_primitives[i].current());
            newCurrent.x = image.cols - 3;
            m_primitives[i].setCurrent(newCurrent);
            GraphVertex v(i, 1, m_primitives[i].current());
            add_vertex(v, m_g);
        }

        if(m_primitives[i].current().y >= image.rows - 3){
            m_primitives[i].setK(0);
            Point2f newCurrent(m_primitives[i].current());
            newCurrent.y = image.rows - 3;
            m_primitives[i].setCurrent(newCurrent);
            GraphVertex v(i, 2, m_primitives[i].current());
            add_vertex(v, m_g);
        }
    }

    priority_queue<Primitive> primitivesPriorityQueue;
    for(int i = 4 ; i < m_primitives.size() ; ++i){
        primitivesPriorityQueue.push(m_primitives[i]);
    }

    while(!primitivesPriorityQueue.empty()){

        Primitive source = primitivesPriorityQueue.top();
        primitivesPriorityQueue.pop();

        if(source.K() == 0) continue;

        int i = source.idPrimitive();

        for(int j = 4; j < m_primitives.size(); ++j){
            if(i == j) continue;
            if(m_primitives[i].certificates().at(j) == 0) continue;
            if(m_primitives[j].certificates().at(i) == 0) continue;
            if(m_primitives[i].idSegment() == m_primitives[j].idSegment()) continue;

            Point2f current = m_primitives[i].current();
            Point2f origin = m_primitives[j].origin();
            Point2f end = m_primitives[j].current();


            Point2f intersection;
            if(lineIntersection(m_primitives[i].origin(), m_primitives[i].current(), m_primitives[j].origin(), m_primitives[j].current(), intersection)){

                m_primitives[i].flipCertificate(j);
                m_primitives[j].flipCertificate(i);

                m_primitives[i].setCurrent(intersection);

                GraphVertex v(i, j, intersection);
                adjacency_list<vecS, vecS, undirectedS, GraphVertex>::vertex_descriptor vertexDescriptor = add_vertex(v, m_g);

                //add_edge(m_primitives[i].lastIntersection(), vertexDescriptor, m_g);
                //add_edge(m_primitives[j].lastIntersection(), vertexDescriptor, m_g);

                m_primitives[i].setLastIntersection(vertexDescriptor);
                m_primitives[j].setLastIntersection(vertexDescriptor);

                m_primitives[i].setK(0);
            }

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

    GraphVertex cornerUpperLeft(-1, -1, Point2f(2, 2));
    GraphVertex cornerUpperRight(-2, -2, Point2f(image.cols - 3, 2));
    GraphVertex cornerLowerRight(-3, -3, Point2f(image.cols - 3, image.rows - 3));
    GraphVertex cornerLowerLeft(-4, -4, Point2f(2, image.rows - 3));

    add_vertex(cornerUpperLeft, m_g);
    add_vertex(cornerUpperRight, m_g);
    add_vertex(cornerLowerRight, m_g);
    add_vertex(cornerLowerLeft, m_g);

    Primitive UpPrim(Point2f(cornerUpperLeft.intersection()), Point2f(cornerUpperRight.intersection()), lines_fld.size() * 2 + 4, 0, 0, -1);
    m_primitives.push_back(UpPrim);

    Primitive RightPrim(Point2f(cornerUpperRight.intersection()), Point2f(cornerLowerRight.intersection()), lines_fld.size() * 2 + 4, 0, 1, -1);
    m_primitives.push_back(RightPrim);

    Primitive BottomPrim(Point2f(cornerLowerRight.intersection()), Point2f(cornerLowerLeft.intersection()), lines_fld.size() * 2 + 4, 0, 2, -1);
    m_primitives.push_back(BottomPrim);

    Primitive LeftPrim(Point2f(cornerLowerLeft.intersection()), Point2f(cornerUpperLeft.intersection()), lines_fld.size() * 2 + 4, 0, 3, -1);
    m_primitives.push_back(LeftPrim);



    int idPrimitive = 4;
    for(int i = 0 ; i < lines_fld.size() ; ++i){
        Point2f origin(lines_fld[i][0], lines_fld[i][1]);
        Point2f end(lines_fld[i][2], lines_fld[i][3]);
        Point2f middle = middleOfSegment(origin, end);
        GraphVertex v(i, i, middle);
        Primitive p0(middle, origin, lines_fld.size() * 2 + 4, 1, idPrimitive, i);
        ++idPrimitive;
        Primitive p1(middle, end, lines_fld.size() * 2 + 4, 1, idPrimitive, i);
        ++idPrimitive;
        m_primitives.push_back(p0);
        m_primitives.push_back(p1);
        adjacency_list<vecS, vecS, undirectedS, GraphVertex>::vertex_descriptor vertexDescriptor = add_vertex(v, m_g);
        p0.setLastIntersection(vertexDescriptor);
        p1.setLastIntersection(vertexDescriptor);
    }

    for(int i = 0 ; i < m_primitives.size() ; ++i){
        for(int j = i + 1 ; j < m_primitives.size() ; ++j){
            if(m_primitives[i].idSegment() == m_primitives[j].idSegment()) continue;
            Point2f intersection;
            if(lineIntersection(m_primitives[i].origin(), m_primitives[i].end(), m_primitives[j].origin(), m_primitives[j].end(), intersection)){

                pair<pair<int, int>, Point2f> p(make_pair(i, j), intersection);
                intersections.insert(p);
                m_primitives[i].flipCertificate(j);
                m_primitives[j].flipCertificate(i);
            }
        }
    }

    for (std::map<pair<int,int>, Point2f>::iterator it=intersections.begin(); it!=intersections.end(); ++it){
        GraphVertex v(it->first.first, it->first.second, it->second);
        adjacency_list<vecS, vecS, undirectedS, GraphVertex>::vertex_descriptor vertexDescriptor = add_vertex(v, m_g);

        //add_edge(m_primitives[it->first.first].lastIntersection(), vertexDescriptor, m_g);
        //add_edge(m_primitives[it->first.second].lastIntersection(), vertexDescriptor, m_g);

        m_primitives[it->first.first].setLastIntersection(vertexDescriptor);
        m_primitives[it->first.second].setLastIntersection(vertexDescriptor);
    }

    double t = 0;
    double delta = 0.5;
    while(isPropagationLeft()){
        //propagation(t, image);
        propagationPriorityQueue(t, image);
        t += delta;
    }

    pair<GRAPH::vertex_iterator, GRAPH::vertex_iterator> v_it = vertices(m_g);
    for(; v_it.first != v_it.second; ++v_it.first){
        GRAPH::vertex_descriptor v = *v_it.first;
        //std::cout << m_g[v].i() << " " << m_g[v].j() << " " << m_g[v].intersection().x << " " << m_g[v].intersection().y << endl;

        circle( image_fld,
                 m_g[v].intersection(),
                 96.0/32.0,
                 Scalar( 0, 0, 255 ),
                 -1,
                 8 );
    }

    for(int i = 0 ; i < m_primitives.size() ; ++i){
        Point2f origin(m_primitives[i].origin());
        Point2f end(m_primitives[i].current());
        line(image_fld, origin, end, CV_RGB(255, 0, 0), 1, LINE_AA);
    }

    imshow("FLD result", image_fld);
    waitKey();
}
