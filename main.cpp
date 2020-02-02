#include "mainwindow.h"
#include "kippi.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    /*QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();*/

    std::string in;
    cv::CommandLineParser parser(argc, argv, "{@input|../samples/data/corridor.jpg|input image}{help h||show help message}");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    in = parser.get<string>("@input");

    Kippi k;
    k.partition(in);
}

/*using namespace std;
using namespace cv;
using namespace cv::ximgproc;
using namespace boost;

#define EPSILON 0.5


/*
 * Chaque Vertex du graphe est soit:
    -Le milieu d'un segment, dans ce cas i == j == id du segment dans lines_fld >= 0 et intersection == milieu du segment
    -L'intersection de deux segment, dans ce cas i != j ou i et j sont les ids des deux segments dans lines_fld. intersection est l'intersection des deux segments
    -Un des quatres coins de l'image, dans ce cas i == j < 0, avec intersection la valeur du coin
*/
/*struct VERTEX{
    int m_i;
    int m_j;
    Point2f m_intersection;
    VERTEX(int i, int j, Point2f intersection) : m_i(i), m_j(j), m_intersection(intersection) {}
};

struct PRIMITIVE{
    Point2f m_origin;
    Point2f m_end;
    Point2f m_current;
    //id du segment dans lines_fld
    int m_idSegment;
    int m_idPrimitive;
    vector<bool> m_certificates;
    int m_K;
    bool m_propagating;
    PRIMITIVE(Point2f origin, Point2f end, int size, int K, int idPrimitive, int idSegment) : m_origin(origin), m_end(end), m_current(end),
        m_K(K), m_propagating(true), m_idPrimitive(idPrimitive), m_idSegment(idSegment) {
        m_certificates = vector<bool>(size, 1);
        m_certificates[m_idPrimitive] = 0;

        Point2f direction = m_end - m_origin;
        double norm = sqrt(direction.x * direction.x + direction.y * direction.y);
        direction.x /= norm;
        direction.y /= norm;
        return direction;
    }

    int nbCertificates() const{
        int nb = 0;
        for(int i = 0; i < m_certificates.size(); ++i){
            if(m_certificates[i] == 0) ++nb;
        }

        return nb;
    }

    Point2f direction(){
        Point2f direction = m_end - m_origin;
        double norm = sqrt(direction.x * direction.x + direction.y * direction.y);
        direction.x /= norm;
        direction.y /= norm;
        return direction;
    }

    bool operator <(const PRIMITIVE & p) const
    {
        return nbCertificates() < p.nbCertificates();
    }
};

typedef adjacency_list<vecS, vecS, undirectedS, VERTEX> GRAPH;

Point2f middleOfSegment(const Point2f &segmentStart, const Point2f &segmentEnd){
    Point2f middle;

    middle.x = (segmentStart.x + segmentEnd.x) / 2;
    middle.y = (segmentStart.y + segmentEnd.y) / 2;

    return middle;
}

//Source: https://github.com/opencv/opencv/blob/master/modules/imgproc/src/min_enclosing_triangle.cpp#L1521-L1531
bool almostEqual(double number1, double number2) {
    return (std::abs(number1 - number2) <= (EPSILON * max(max(1.0, std::abs(number1)), std::abs(number2))));
}

double distanceBetweenPoints(const Point2f &a, const Point2f &b){
    double xDiff = b.x - a.x;
    double yDiff = b.y - a.y;

    return sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

bool arePointsColinear(const Point2f &a, const Point2f &b, const Point2f &c){
    double crossproduct = (c.y - a.y) * (b.x - a.x) - (c.x - a.x) * (b.y - a.y);
    if(abs(crossproduct) > EPSILON) return false;
    return true;
}

bool pointOnSegment(const Point2f &segmentStart, const Point2f &segmentEnd, const Point2f &toEvaluate){
    double dStart = distanceBetweenPoints(segmentStart, toEvaluate);
    double dEnd = distanceBetweenPoints(toEvaluate, segmentEnd);
    double segmentLength = distanceBetweenPoints(segmentStart, segmentEnd);

    return (almostEqual(dStart + dEnd, segmentLength));
}


//Source: https://github.com/opencv/opencv/blob/master/modules/imgproc/src/min_enclosing_triangle.cpp#L1380-L1424
bool lineIntersection(const Point2f &a1, const Point2f &b1, const Point2f &a2,
                             const Point2f &b2, Point2f &intersection) {
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

bool isPropagationLeft(vector<PRIMITIVE> &primitives){
    for(int i = 0; i < primitives.size(); ++i){
        if(primitives[i].m_K != 0) return true;
    }
    return false;
}

void propagation(double t, vector<PRIMITIVE> &primitives, GRAPH &g, const Mat &img){
    for(int i = 0; i < primitives.size(); ++i){
        if(primitives[i].m_K == 0) continue;
        Point2f end(primitives[i].m_end);
        Point2f direction = primitives[i].direction();
        end.x += direction.x * t;
        end.y += direction.y * t;
        primitives[i].m_current = end;
        if(primitives[i].m_current.x < 0 ||primitives[i].m_current.y < 0 ||
                primitives[i].m_current.x > img.cols || primitives[i].m_current.y > img.rows) primitives[i].m_K = 0;
    }

    for(int i = 0; i < primitives.size(); ++i){
        if(primitives[i].m_K == 0) continue;
        for(int j = 0; j < primitives.size(); ++j){
            if(i == j) continue;
            if(primitives[i].m_idSegment == primitives[j].m_idSegment) continue;
            Point2f current = primitives[i].m_current;
            Point2f origin = primitives[j].m_origin;
            Point2f end = primitives[j].m_current;

            if(!arePointsColinear(origin, end, current)) continue;
            if(!pointOnSegment(origin, end, current)) continue;

            primitives[i].m_certificates[j] = 0;

            VERTEX v(i, j, current);
            add_vertex(v, g);

            primitives[i].m_K--;
        }
    }
}

/*void propagationPriorityQueue(double t, priority_queue<PRIMITIVE> &primitivesPriorityQueue, vector<PRIMITIVE> &primitives, GRAPH &g, const Mat &img){
    PRIMITIVE source = primitivesPriorityQueue.top();
    primitivesPriorityQueue.pop();

    if(source.m_K == 0) return;

    Point2f end(source.m_end);
    Point2f direction = source.direction();
    end.x += direction.x * t;
    end.y += direction.y * t;
    primitives[source.m_idPrimitive].m_current = end;

    if(primitives[source.m_idPrimitive].m_current.x < 0 ||primitives[source.m_idPrimitive].m_current.y < 0 ||
            primitives[source.m_idPrimitive].m_current.x > img.cols || primitives[source.m_idPrimitive].m_current.y > img.rows)
                primitives[source.m_idPrimitive].m_K = 0;

    for(int i = 0; i < primitives.size(); ++i){
        if(i == source.m_idPrimitive) continue;
        Point2f current = primitives[source.m_idPrimitive].m_current;
        Point2f origin = primitives[i].m_origin;
        Point2f end = primitives[i].m_current;
        if(!arePointsColinear(origin, end, current)) continue;
        if(!pointOnSegment(origin, end, current)) continue;

        primitives[source.m_idPrimitive].m_certificates[i] = 0;

        VERTEX v(source.m_idPrimitive, i, current);
        add_vertex(v, g);

        primitives[source.m_idPrimitive].m_K--;
    }
}*/

/*int main(int argc, char** argv)
{
    std::string in;
    cv::CommandLineParser parser(argc, argv, "{@input|../samples/data/corridor.jpg|input image}{help h||show help message}");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    in = parser.get<string>("@input");
    Mat image = imread(in, IMREAD_GRAYSCALE);
    if( image.empty() )
    {
        return -1;
    }
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
    //vector<vector<Point2f>> lines;
    fld->detect(image, lines_fld);
    Mat line_image_fld(image);

    map<pair<int, int>, Point2f> intersections;

    typedef adjacency_list<vecS, vecS, undirectedS, VERTEX> GRAPH;

    vector<PRIMITIVE> primitives;
    GRAPH g;
    VERTEX cornerUpperLeft(-1, -1, Point2f(0, 0));
    VERTEX cornerUpperRight(-2, -2, Point2f(0, image.cols));
    VERTEX cornerLowerRight(-3, -3, Point2f(image.rows, image.cols));
    VERTEX cornerLowerLeft(-4, -4, Point2f(image.rows, 0));

    add_vertex(cornerUpperLeft, g);
    add_vertex(cornerUpperRight, g);
    add_vertex(cornerLowerRight, g);
    add_vertex(cornerLowerLeft, g);

    int idPrimitive = 0;
    for(int i = 0 ; i < lines_fld.size() ; ++i){
        Point2f origin0(lines_fld[i][0], lines_fld[i][1]);
        Point2f end0(lines_fld[i][2], lines_fld[i][3]);
        Point2f middle = middleOfSegment(origin0, end0);
        VERTEX v(i, i, middle);
        PRIMITIVE p0(middle, origin0, lines_fld.size(), 1, idPrimitive, i);
        ++idPrimitive;
        PRIMITIVE p1(middle, end0, lines_fld.size(), 1, idPrimitive, i);
        ++idPrimitive;
        primitives.push_back(p0);
        primitives.push_back(p1);
        add_vertex(v, g);
    }

    for(int i = 0 ; i < primitives.size() ; ++i){
        for(int j = i + 1 ; j < primitives.size() ; ++j){
            //if(primitives[i].m_idSegment == primitives[j].m_idSegment) continue;
            Point2f intersection;
            if(lineIntersection(primitives[i].m_origin, primitives[i].m_end, primitives[j].m_origin, primitives[j].m_end, intersection)){
                pair<pair<int, int>, Point2f> p(make_pair(i, j), intersection);
                intersections.insert(p);
                primitives[i].m_certificates[j];
                primitives[j].m_certificates[i];
            }
        }
    }

    for (std::map<pair<int,int>, Point2f>::iterator it=intersections.begin(); it!=intersections.end(); ++it){
        VERTEX v(it->first.first, it->first.second, it->second);
        add_vertex(v, g);
    }

    double t = 0;
    double delta = 0.1;
    while(isPropagationLeft(primitives)){
        priority_queue<PRIMITIVE> primitivesPriorityQueue;
        for(int i = 0 ; i < primitives.size() ; ++i){
            if(primitives[i].m_K > 0)
                primitivesPriorityQueue.push(primitives[i]);
        }
        propagation(t, primitives, g, line_image_fld);
        //propagationPriorityQueue(t, primitivesPriorityQueue, primitives, g, line_image_fld);
        t += delta;
    }

    pair<GRAPH::vertex_iterator, GRAPH::vertex_iterator> v_it = vertices(g);
    for(; v_it.first != v_it.second; ++v_it.first){
        GRAPH::vertex_descriptor v = *v_it.first;
        std::cout << g[v].m_i << " " << g[v].m_j << " " << g[v].m_intersection.x << " " << g[v].m_intersection.y << endl;
    }


    //fld->drawSegments(line_image_fld, lines_fld);

    for(int i = 0 ; i < primitives.size() ; ++i){
        Point2f origin(primitives[i].m_origin);
        Point2f end(primitives[i].m_current);
        line(line_image_fld, origin, end, CV_RGB(255, 0, 0), 1, LINE_AA);
    }

    imshow("FLD result", line_image_fld);
    waitKey();
    return 0;
}*/
