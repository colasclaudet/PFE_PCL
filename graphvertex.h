#ifndef GRAPHVERTEX_H
#define GRAPHVERTEX_H

#include <opencv2/core/types.hpp> //Point2f
#include <set>

using namespace std;
using namespace cv;

/**
 * @brief Classe modélisant une intersection entre deux primitives. Utilisée
 * dans le Graphe Kippi::m_g comme structure de stockage des données des vertices
 * du graphe.
 */
class GraphVertex
{

private:
    /**
     * @brief m_i: Index dans Kippi::m_primitives de la première primitive
     * impliquée dans l'intersection.
     */
    int m_i;

    /**
     * @brief m_j: Index dans Kippi::m_primitives de la seconde primitive
     * impliquée dans l'intersection
     */
    int m_j;

    /**
     * @brief m_intersection: Point d'intersection entre les deux primitives
     */
    Point2f m_intersection;

    /**
     * @brief m_index: Index du vertex dans le graphe
     */
    int m_index;

    /**
     * @brief m_active: false si le sommet a été supprimé. Evite d'avoir à gérer
     * la modification des index de tous les autres sommets en cas de surpression
     */
    bool m_active = true;

public:
    /**
     * @brief Constructeur de la classe GraphVertex
     * @param i: Index dans Kippi::m_primitives de la première primitive
     * impliquée dans l'intersection.
     * @param j: Index dans Kippi::m_primitives de la seconde primitive
     * impliquée dans l'intersection.
     * @param intersection: Point d'intersection entre les deux primitives
     */
    GraphVertex(int i, int j, Point2f intersection, int index);

    /**
     * @brief Constructeur par défaut de la classe GraphVertex. Nécéssaire pour
     * l'utilisation de celle-ci dans un graphe Boost
     */
    GraphVertex();

    /**
     * @brief Constructeur par recopie de la classe GraphVertex. Nécéssaire pour
     * l'utilisation de celle-ci dans un graphe Boost
     * @param g: Objet GraphVertex à recopier.
     */
    GraphVertex(const GraphVertex &g);

    /**
     * @brief Getter de m_i
     * @return int m_i
     */
    int i() const;

    /**
     * @brief Getter de m_j
     * @return int m_j
     */
    int j() const;

    /**
     * @brief Getter de m_intersection
     * @return Point2f m_intersection
     */
    Point2f intersection() const;
    int index() const;
    void setIndex(int index);
    set<int> adjacentVertices;
    void flipActive();
};

#endif // GRAPHVERTEX_H

