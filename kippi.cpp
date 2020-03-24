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
    return (abs(number1 - number2) <= (EPSILON * max(max(1.0, abs(number1)), abs(number2))));
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

void Kippi::propagationPriorityQueue(double t, const Mat &image){
    for(int i = 0; i < m_primitives.size(); ++i){
        if(m_primitives[i].K() == 0) continue;
        Point2f end(m_primitives[i].end());
        Point2f direction = m_primitives[i].direction();
        end.x += direction.x * t;
        end.y += direction.y * t;
        m_primitives[i].setCurrent(end);

        if(m_primitives[i].current().x < 0){
            m_primitives[i].setK(0);
            Point2f newCurrent(m_primitives[i].current());
            newCurrent.x = 0;
            m_primitives[i].setCurrent(newCurrent);
        }

        if(m_primitives[i].current().y < 0){
            m_primitives[i].setK(0);
            Point2f newCurrent(m_primitives[i].current());
            newCurrent.y = 0;
            m_primitives[i].setCurrent(newCurrent);
        }

        if(m_primitives[i].current().x > image.cols){
            m_primitives[i].setK(0);
            Point2f newCurrent(m_primitives[i].current());
            newCurrent.x = image.cols - 1;
            m_primitives[i].setCurrent(newCurrent);
        }

        if(m_primitives[i].current().y > image.rows){
            m_primitives[i].setK(0);
            Point2f newCurrent(m_primitives[i].current());
            newCurrent.y = image.rows - 1;
            m_primitives[i].setCurrent(newCurrent);
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

        for(int j = 0; j < m_primitives.size(); ++j){
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

                m_primitives[i].setK(0);
            }

        }

    }
}

pair<vector<vector<QVector2D>>, vector<double>> Kippi::partition(string imgName, bool display){
    Mat image = imread(imgName, IMREAD_GRAYSCALE);
    if( image.empty() )
    {
        exit(0);
    }

    Mat imgCopy = Mat(image.rows, image.cols, CV_8UC3, Scalar(255,255, 255));
    cvtColor(image, imgCopy, COLOR_GRAY2RGB);

    Mat image_fld = Mat::zeros( image.size(), CV_8UC1 );
    image_fld.setTo(cv::Scalar(255));
    Mat linesImg = Mat(image.rows, image.cols, CV_8UC3, Scalar(255,255, 255));

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
    int length_threshold = 15;
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

    int idPrimitive = 0;
    for(int i = 0 ; i < lines_fld.size() ; ++i){
        Point2f origin(lines_fld[i][0], lines_fld[i][1]);
        Point2f end(lines_fld[i][2], lines_fld[i][3]);
        Point2f middle = middleOfSegment(origin, end);
        Primitive p0(middle, origin, lines_fld.size() * 2 + 4, 1, idPrimitive, i);
        ++idPrimitive;
        Primitive p1(middle, end, lines_fld.size() * 2 + 4, 1, idPrimitive, i);
        ++idPrimitive;
        m_primitives.push_back(p0);
        m_primitives.push_back(p1);
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

    double t = 0;
    double delta = 0.5;
    while(isPropagationLeft()){
        propagationPriorityQueue(t, image);
        t += delta;
    }

    for(int i = 0 ; i < m_primitives.size() ; ++i){
        Point2f origin(m_primitives[i].origin());
        Point2f end(m_primitives[i].current());
        line(linesImg, origin, end, CV_RGB(0, 0, 0), 1, 4);
        line(imgCopy, origin, end, CV_RGB(255,0,0), 1, 4);
    }

    Mat linesGray;

    cvtColor(linesImg, linesGray, COLOR_BGR2GRAY );
    threshold(linesGray, linesGray, 0, 255, THRESH_BINARY + THRESH_OTSU);

    Mat canny_output;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;


    int thresh = 50;
    /// Detect edges using canny
    Canny( linesGray, canny_output, thresh, thresh*2, 5 );
    /// Find contours
    findContours( linesGray, contours, hierarchy, RETR_TREE,
                  CHAIN_APPROX_TC89_L1, Point(1, 1) );


    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC1 );

    vector<double> medianValues;
    vector<vector<QVector2D>> coordinates;

    for( int i = 0; i < contours.size(); i++ )
    {
        vector<QVector2D> pointsContours;
        vector<pair<int, int>> toRecolor;

            for(int j = 0 ; j < contours[i].size() ; j++){
                QVector2D point(contours[i][j].x, contours[i][j].y);
                pointsContours.push_back(point);
            }

            vector<int> colorValues;
            Mat drawing = Mat::zeros( canny_output.size(), CV_8UC1 );
            Scalar color = Scalar(255);
            drawContours( drawing, contours, i, color, FILLED, LINE_AA );

            for(int y = 0 ; y < drawing.cols ; ++y){
                for(int x = 0 ; x < drawing.rows ; ++x){
                    Scalar intensity = drawing.at<uchar>(Point(y, x));
                    int intensityValue = intensity.val[0];
                    if(intensityValue != 0){
                        Scalar imageIntensity = image.at<uchar>(Point(y, x));
                        int imageIntensityValue = imageIntensity.val[0];
                        if(imageIntensityValue != 0){
                            colorValues.push_back(imageIntensityValue);
                        }
                        pair<int, int> p(x, y);
                        toRecolor.push_back(p);
                    }
                }
            }

            sort(colorValues.begin(), colorValues.end());
            double medianValue = 0.0f;

            if(colorValues.size() == 0) continue;
            if(colorValues.size() == 1) {
                medianValue = colorValues[0];
            }
            else{
                if (colorValues.size() % 2 == 0) {
                  medianValue = (colorValues[colorValues.size() / 2 - 1] + colorValues[colorValues.size() / 2]) / 2;
                }

                else {
                  medianValue = colorValues[floor(colorValues.size() / 2)];
                }
            }

            for(int j = 0 ; j < toRecolor.size() ; ++j){
                pair<int, int> current = toRecolor[j];
                image_fld.at<uchar>(current.first, current.second) = medianValue;
            }

            medianValues.push_back(medianValue);
            coordinates.push_back(pointsContours);
    }

    if(display){
        imshow("Lines", imgCopy);
        imshow("FLD result", linesGray);
        imshow("Median Result", image_fld);
    }


    pair<vector<vector<QVector2D>>, vector<double>> contoursValues(coordinates, medianValues);
    return contoursValues;
}
