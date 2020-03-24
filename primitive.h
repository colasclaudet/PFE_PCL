#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include <cmath> //sqrt
#include <vector> //vector
#include <opencv2/core/types.hpp> //Point2f

using namespace std;
using namespace cv;

/**
 * @brief Classe modélisant un demi-segment de l'image, s'étendant dans une
 * direction donnée (m_direction). Modélise aussi, en cas particuliers, les
 * bords de l'image, qui ne s'étendent pas mais sont plus simple à traiter en
 * tant que Primitives dans la logique de l'application.
 */
class Primitive
{
private:
    /**
     * @brief m_origin: Point d'origine de la primitive. Chaque segment de
     * l'image est coupée en deux à son milieu pour former deux primitives,
     * l'origine de chaque primitive est donc le milieu du segment originel (sauf
     * pour les bords de l'image)
     */
    Point2f m_origin;

    /**
     * @brief m_end: Point de fin originel de la primitive, correspondant à une
     * des deux extrémités du segment originel
     */
    Point2f m_end;

    /**
     * @brief m_current: Point de fin courant de la primitive lors de son
     * extension. Initialisé à m_end au départ
     */
    Point2f m_current;

    /**
     * @brief m_direction: Direction d'extension de la primitive
     */
    Point2f m_direction;

    /**
     * @brief m_idSegment: Index du segment dont fait partie la primitive dans
     * le tableau lines_fld utilisée dans Kippi (tableau résultant de la
     * recherche de segment par l'algorithme fast line detector d'OpenCV)
     */
    int m_idSegment;

    /**
     * @brief m_idPrimitive: Index de la primitive dans le tableau
     * Kippi::m_primitives
     */
    int m_idPrimitive;

    /**
     * @brief m_certificates: Tableau référençant si la primitive courante est
     * déjà entrée en intersection avec chaque autre primitive
     */
    vector<bool> m_certificates;

    /**
     * @brief m_K: Nombre d'intersections données à la primitive avant que la
     * propagation de celle-ci s'arrête
     */
    int m_K;
public:
    /**
     * @brief Constructeur de la classe primitive
     * @param origin: Point d'origine de la primitive
     * @param end: Point de fin originel de la primitive
     * @param size: Taille du tableau m_certificates (nombre total de primitives
     * dans l'image)
     * @param K: Nombre d'intersections données à la primitive avant que la
     * propagation de celle-ci s'arrête
     * @param idPrimitive: Index de la primitive dans le tableau
     * Kippi::m_primitives
     * @param idSegment: Index du segment dont fait partie la primitive dans
     * le tableau lines_fld utilisée dans Kippi
     */
    Primitive(Point2f origin, Point2f end, int size, int K, int idPrimitive,
              int idSegment);

    /**
     * @brief Nombre de certificats validés par la primitive (= nombre de fois
     * ou elle est entrée en intersection avec une autre primitive)
     */
    int nbCertificates() const;

    /**
     * @brief Surcharge de l'opérateur de comparaison <. Utilisée pour le tri
     * d'une priority_queue de Primitives dans Kippi
     * @param p: Primitive à comparer à la primitive courante
     */
    bool operator <(const Primitive & p) const;

    /**
     * @brief Getter de m_origin
     * @return Point2f m_origin
     */
    Point2f origin() const;

    /**
     * @brief Getter de m_end
     * @return Point2f m_end
     */
    Point2f end() const;

    /**
     * @brief Getter de m_curren
     * @return Point2f m_current
     */
    Point2f current() const;

    /**
     * @brief Setter de m_current
     * @param current: Nouvelle position à  affecter à m_current
     */
    void setCurrent(const Point2f &current);

    /**
     * @brief Getter de m_direction
     * @return Point2f m_direction
     */
    Point2f direction() const;

    /**
     * @brief Getter de m_idSegment
     * @return int m_idSegment
     */
    int idSegment() const;

    /**
     * @brief Getter de m_idPrimitive
     * @return int m_idPrimitive
     */
    int idPrimitive() const;

    /**
     * @brief Getter de m_certificates
     * @return vector<bool> m_certificates
     */
    vector<bool> certificates();

    /**
     * @brief Inverse la valeur du certificat dans m_certificates à l'index
     * passé en paramètre
     * @param index: Index auquel inverser le certificat
     */
    void flipCertificate(int index);

    /**
     * @brief Getter de m_K
     * @return int m_K
     */
    int K() const;

    /**
     * @brief Setter de m_K
     * @param K: Nouvelle valeur à affecter à m_K
     */
    void setK(int K);
};

#endif // PRIMITIVE_H
