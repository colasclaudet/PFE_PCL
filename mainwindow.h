#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <chrono>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <thread>
#include <QMessageBox>
#include <float.h>
#include <QVector3D>
#include <math.h>
#include "fragment.h"
#include <iostream>


/********** INCLUDE PCL *********************/

//#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/features/impl/normal_3d_omp.hpp>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/ply_io.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/io/vtk_io.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>

#include <vtkRenderWindow.h>

#include <string>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/don.h>
#include <pcl/common/point_operators.h>
#include <pcl/search/octree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


#include <QThread>

//#include <pcl/common/impl/intersections.hpp>
//#include <pcl/common/impl/intersections.hpp>
/********** FIN INCLUDE PCL *********************/
#include "ui_mainwindow.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow, private Ui::MainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void showHelp(char *program_name);
    /**********************
      VISUALISATION
    **********************/
    pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    pcl::visualization::PCLVisualizer::Ptr simpleVisCol(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color);
    pcl::visualization::PCLVisualizer::Ptr addVisualiser();
    pcl::visualization::PCLVisualizer::Ptr finaliseVis(pcl::visualization::PCLVisualizer::Ptr viewer);
    pcl::visualization::PCLVisualizer::Ptr addPtsCloudColor(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color);
    pcl::visualization::PCLVisualizer::Ptr addPtsCloud (pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
    static void showVizualizer();
    /**********************
          DEBRUITAGE
    **********************/
    void denoise(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    /**********************
          MODELISATION
    **********************/
    pcl::visualization::PCLVisualizer::Ptr addPlane (pcl::visualization::PCLVisualizer::Ptr viewer, pcl::ModelCoefficients planeCoef);
    pcl::PointXYZ threePlaneIntersection(pcl::ModelCoefficients plane_coeff1, pcl::ModelCoefficients plane_coeff2, pcl::ModelCoefficients plane_coeff3);
    pcl::PointCloud<pcl::PointXYZ> clouds_union(std::vector<pcl::PointCloud<pcl::PointXYZ>> v_cloud);
    /**********************
           CALCUL
    **********************/
    void repereRoom(pcl::visualization::PCLVisualizer::Ptr viewer, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> list_planes);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int degrees, int axe);

    /**********************
           CALCUL
    **********************/
    void repereRoom(pcl::visualization::PCLVisualizer::Ptr viewer, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> list_planes);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int degrees, int axe);
    void searchLimit (std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> list_planes, std::vector<double *> list_equat_planes, std::vector<QVector3D> list_normale_planes, std::vector<QVector3D> list_meanPosition);
    QVector3D computeNormalePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane);
    QVector3D computeMeanPositionPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane);
    double computeDistance(QVector3D p1, QVector3D p2);
    bool normalesAreSimilar(QVector3D p1, QVector3D p2);
    int axeViaNormal(QVector3D n);
    void createRoom();

    /**********************
           OBSOLETE
    **********************/
    pcl::PointCloud<pcl::PointXYZ>::Ptr regroup_plane(); //not use

    /**********************
              PFE
     *********************/

    /**********************
     resolve 3 parameters equation
     **********************/
    QVector3D resol_3eq_3inc(double * eq1, double * eq2, double * eq3);
    /**********************
     get plane equation
     **********************/
    double * equation_plane(QVector3D P1, QVector3D P2, QVector3D P3);
    double * equation_plane2(pcl::PointCloud<pcl::PointXYZ> pcloud);

    double * moy_eq_plane(pcl::PointCloud<pcl::PointXYZ> pcloud);

    /**********************
            Add xyzrgb point cloud
     *********************/
    pcl::visualization::PCLVisualizer::Ptr addPtsCloudXYZRGB (pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    pcl::PointCloud<pcl::PointXYZRGB> clouds_union(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> v_cloud);

    /**********************
              Direction Of Normal Segmentation
     *********************/


    void don_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, double angle, double threshold,double scale1, double scale2);

    void ransac_segmentation();
    void calc_bounding_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void calc_inter_planes();
public slots:
    void chooseViewCloud();
    void chooseViewPlane();
    void changeThreshold(int th);
    void changeProba(int proba);
    void chooseFile();
    void draw();
    void modelize();
    void saveCloud(); //not use

private:
    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; //(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final; //(new pcl::PointCloud<pcl::PointXYZ>);

    bool file_is_ply = false;

    Ui::MainWindow *ui;
    float threshold = 90.0f;
    float proba = 0.05f;
    bool view_plan = true;
    bool have_plane = false;


    std::vector<pcl::PointCloud<pcl::PointXYZ>> vector_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToSave;
    std::vector<pcl::ModelCoefficients> vector_eq;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> vector_cloud_RGB;

    std::vector<Fragment> list_limits;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> room;

    std::vector<double *> eq_planes;
    std::vector<QVector3D> inter_points;
    QString file = "";

    int nb_cloud = 0;
    int nb_plane = 0;

    double xmin,ymin,zmin = 10000000000;
    double xmax,ymax,zmax = -10000000000;


    QThread processViewer;

//new
protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;

private slots:
    void on_action_propos_triggered();
};

#endif // MAINWINDOW_H
