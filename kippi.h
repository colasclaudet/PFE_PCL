#ifndef KIPPI_H
#define KIPPI_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp> //Dépendance OpenCV
#include <opencv2/imgproc.hpp> //Dépendance OpenCV
#include "opencv2/ximgproc.hpp" //fast line detector
#include <opencv2/core/utility.hpp> //Dépendance OpenCV
#include <opencv2/features2d.hpp> //Dépendance OpenCV
#include <opencv2/highgui.hpp> //Dépendance OpenCV
#include <vector> //vector
#include <map> //map
#include <cmath> //sqrt, abs
#include <boost/graph/graph_traits.hpp> //vertex_descriptor
#include <boost/graph/adjacency_list.hpp> //adjacency_list
#include <queue> //priority_queue
#include "primitive.h"
#include "graphvertex.h"

using namespace std;
using namespace cv;
using namespace cv::ximgproc;
using namespace boost;

//Définition d'une marge d'approximation pour les égalités.
#define EPSILON 1E-10



/**
 * @brief Classe pirincipale de la partie Kippi de notre application. Regroupe
 * les fonction nécéssaires à l'éxécution de cet algorithme
 */
class Kippi
{
private:
    /**
     * @brief GRAPH: Définition du type de graphe Boost que nous allons utiliser
     */
    typedef adjacency_list<listS, listS, undirectedS, GraphVertex> GRAPH;

    /**
     * @brief m_primitives: Tableau stockant toutes les primitives de l'image
     */
    vector<Primitive> m_primitives;

    /**
     * @brief Retourne les coordonnées du milieu d'un segment
     * @param segmentStart: Point de départ du segment
     * @param segmentEnd: Point de fin du segment
     */
    Point2f middleOfSegment(const Point2f &segmentStart, const Point2f &segmentEnd);

    /**
     * @brief Opérateur égalité à EPSILON près.
     * Source: https://github.com/opencv/opencv/blob/master/modules/imgproc/src/min_enclosing_triangle.cpp#L1521-L1531
     * @param number1: Premier nombre à comparer
     * @param number2: Second nombre à comparer
     */
    bool almostEqual(double number1, double number2);

    /**
     * @brief Retourne la distance entre deux Point2f
     * @param a: Premier point
     * @param b: Second Point
     */
    double distanceBetweenPoints(const Point2f &a, const Point2f &b);

    /**
     * @brief Teste la colinéarité de 3 points
     * @param a: Premier point
     * @param b: Second point
     * @param c: Troisième point
     */
    bool arePointsColinear(const Point2f &a, const Point2f &b, const Point2f &c);

    /**
     * @brief Teste si le point toEvaluate est sur le segment segmentStart-
     * segmentEnd, en fonction de la colinéarité des trois points et de la
     * distance entre ceux-ci
     * @param segmentStart: Point de départ du segment
     * @param segmentEnd: Point de fin du segment
     * @param toEvaluate: Point à évaluer
     */
    bool pointOnSegment(const Point2f &segmentStart, const Point2f &segmentEnd,
                        const Point2f &toEvaluate);

    /**
     * @brief Teste l'intersection de deux segments.
     * Source (modifiée pour ajouter des tests d'appartenance aux segments, la
     * fonction de base étant pour des droites:
     * https://github.com/opencv/opencv/blob/master/modules/imgproc/src/min_enclosing_triangle.cpp#L1380-L1424
     *
     * @param a1: Point de départ du premier segment
     * @param b1: Point de fin du premier segment
     * @param a2: Point de départ du second segment
     * @param b2: Point de fin du second segment
     * @param intersection: Point vide afin de stocker l'intersection
     */
    bool lineIntersection(const Point2f &a1, const Point2f &b1, const Point2f &a2,
                            const Point2f &b2, Point2f &intersection);

    /**
     * @brief Teste si il reste des primitives à propager dans l'image
     */
    bool isPropagationLeft();

    //void propagation(double t, const Mat &image);

    /**
     * @brief Fonction de propagation des primitives dans l'image en fonction du
     * temps t
     * @param t: Temps écoulé
     * @param image: Image d'origine des primitives
     */
    void propagationPriorityQueue(double t, const Mat &image);

public:
    /**
     * @brief Constructeur par défaut de la classe Kippi
     */
    Kippi();

    /**
     * @brief Getter de m_primitives
     * @return vector<Primitive> m_primitives
     */
    vector<Primitive> primitives();

    /**
     * @brief m_g: Graph Boost référencant les intersections entre primitives et
     * le lien entre celles-ci
     */
    GRAPH m_g;

    /**
     * @brief Partitionne l'image passée en paramètre selon une implémentation de
     * l'algorithme Kippi: https://hal.inria.fr/hal-01740958/file/kippi.pdf
     * @param imgName: Chemin de l'image à partitionner
     * @param display: True: Affiche la partition de l'image
     */
    pair<vector<vector<Point>>, vector<double>> partition(string imgName, bool display = false);
};

#endif // KIPPI_H
