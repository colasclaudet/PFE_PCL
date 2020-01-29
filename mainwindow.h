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

/********** INCLUDE PCL *********************/

#include <iostream>
#include <thread>
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
    /**********************
          DEBRUITAGE
    **********************/
    void denoise(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    /**********************
          MODELISATION
    **********************/
    pcl::visualization::PCLVisualizer::Ptr addPlane (pcl::visualization::PCLVisualizer::Ptr viewer, pcl::ModelCoefficients planeCoef);
    pcl::PointXYZ threePlaneIntersection(pcl::ModelCoefficients plane_coeff1, pcl::ModelCoefficients plane_coeff2, pcl::ModelCoefficients plane_coeff3);
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
    /**********************
            Add xyzrgb point cloud
     *********************/
    pcl::visualization::PCLVisualizer::Ptr addPtsCloudXYZRGB (pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    /**********************
              Direction Of Normal Segmentation
     *********************/


    void don_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, double angle, double threshold,double scale1, double scale2);

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
    Ui::MainWindow *ui;
    float threshold = 20.0f;
    float proba = 0.05f;
    bool view_plan = true;	
	bool have_plane = false;

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vector_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToSave;
	std::vector<pcl::ModelCoefficients> vector_eq;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> vector_cloud_RGB;
    QString file = "";
    
    int nb_cloud = 0;
    int nb_plane = 0;
//new
protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;

};

#endif // MAINWINDOW_H
