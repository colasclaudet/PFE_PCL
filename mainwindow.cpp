//Create with the help of http://pointclouds.org/documentation/tutorials/random_sample_consensus.php

#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);
    this->setWindowTitle("ScanLIDAR");
    //new


    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer->getRenderWindow();

    /*
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();*/
    //endNew
	connect(ui->radio_cloud, SIGNAL(clicked()), this, SLOT(chooseViewCloud())); //si cloud radio button sélectionné
	connect(ui->radio_plane, SIGNAL(clicked()), this, SLOT(chooseViewPlane())); //si plane radio button selectionné
	connect(ui->slider_threshold, SIGNAL(valueChanged(int)), this, SLOT(changeThreshold(int))); //connexion slidebar threshold
	connect(ui->slider_proba, SIGNAL(valueChanged(int)), this, SLOT(changeProba(int))); //connexion slidebar proba
	connect(ui->btn_import, SIGNAL(clicked()), this, SLOT(chooseFile())); //connexion selection de fichier
	connect(ui->btn_draw, SIGNAL(clicked()), this, SLOT(draw())); //connexion bouton draw, on lance le viewer
	//save
    //connect(ui->btn_save, SIGNAL(clicked()), this, SLOT(saveCloud()));

}

MainWindow::~MainWindow()
{
	delete ui;
}

/**********************************
*
*          PCL
*
**********************************/

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//SLOT - sauvegarde du nuage de point dans un fichier ply
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

void MainWindow::saveCloud(){
	QString fileName;
	fileName = QFileDialog::getSaveFileName(this,
	tr("Sauvegarder votre image"), "cloud_plane.ply",
	tr("Polygon File Format(.ply);;Tous les fichiers()"));
	if(this->cloudToSave->size() == 0){
		QMessageBox msgBox;
		msgBox.setText("The cloud is empty. Save impossible.");
		msgBox.exec();
	}
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ> (fileName.toStdString(), *this->cloudToSave, false);
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//aide si mauvais arguments
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
void showHelp(QString program_name)
{
	qDebug() << "Usage: " << program_name << " cloud_filename.[ply]";
	qDebug() << "-h:  Show this help.";
}
/**********************
      VISUALISATION
**********************/
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//création d'un viewer affichant un nuage de point passé en paramètre avec fond noir et points blancs
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

pcl::visualization::PCLVisualizer::Ptr MainWindow::simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//viewer->addCoordinateSystem (1.0, "global");
	viewer->initCameraParameters ();
	return (viewer);
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//création d'un viewer affichant un nuage de point passé en paramètre avec fond noir et couleur des points passées en paramètre
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

pcl::visualization::PCLVisualizer::Ptr MainWindow::simpleVisCol (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
single_color)
{
	// -----------------------------------------------
	// -----Open 3D viewer and add color point cloud--
	// -----------------------------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	//viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

	//viewer->addCoordinateSystem (1.0, "global");
	viewer->initCameraParameters ();
	return (viewer);
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//construction et initialisation d'un viewer
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

pcl::visualization::PCLVisualizer::Ptr MainWindow::addVisualiser()
{
	// ---------------------
	// -----set 3D viewer --
	// ---------------------
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);

	return (viewer);
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//Finalise la construction du viewer après l'ajout de nuages de points par addPtsCloudColor
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

pcl::visualization::PCLVisualizer::Ptr MainWindow::finaliseVis (pcl::visualization::PCLVisualizer::Ptr viewer)
{
	for(int i=0;i<this->nb_cloud;i++)
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"+i);

	//viewer->addCoordinateSystem (1.0, "global");
	viewer->initCameraParameters ();
	return (viewer);
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//ajoute un nuage de points au viewer avec une couleur donnée
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

pcl::visualization::PCLVisualizer::Ptr MainWindow::addPtsCloudColor (pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
single_color)
{
	// -----------------------------------------------
	// -----Open 3D viewer and add color point cloud--
	// -----------------------------------------------

	viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud"+this->nb_cloud);
	this->nb_cloud = this->nb_cloud +1;
	return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr MainWindow::addPtsCloud (pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
	// -----------------------------------------------
	// -----Open 3D viewer and add color point cloud--
	// -----------------------------------------------

	//viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud"+this->nb_cloud);
	this->nb_cloud = this->nb_cloud +1;
	return (viewer);
}
/**********************
   FIN VISUALISATION
**********************/

/**********************
      MODELISATION
**********************/
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//Ajout d'un plan au viewer avec en paramètre une équation de plan
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

pcl::visualization::PCLVisualizer::Ptr  MainWindow::addPlane(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::ModelCoefficients planeCoef)
{
	this->nb_plane = this->nb_plane +1;
	viewer->addPlane(planeCoef, planeCoef.values[0],planeCoef.values[1], planeCoef.values[2], "plane"+this->nb_plane,0);
	//viewer->addPlane(planeCoef);
    return (viewer);
}
/**********************
      PFE
**********************/
pcl::visualization::PCLVisualizer::Ptr MainWindow::don_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, double angle, double threshold, double scale1, double scale2)
{
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;
    if (cloud->isOrganized ())
    {
        tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB> ());
    }
    else
    {
        tree.reset(new pcl::search::KdTree<pcl::PointXYZRGB> (false));
    }
    //pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudcolor; //attention nous devons faire une copie de notre ptcl dans ce ptcl de couleur
    //Init tree with pointcloud
    //copyPointCloud (*cloudcolor, *cloud);
    tree->setInputCloud(cloud);

    if (scale1 >= scale2)
    {
        std::cerr << "Error: Large scale must be > small scale!" << std::endl;
        //exit (EXIT_FAILURE);
        if(scale1>scale2)
        {
            double temp = scale2;
            scale2 = scale1;
            scale1 = temp;
        }
        else
        {
            std::cerr << "Error: scale1 and scale2 have the same value!" << std::endl;
            exit (EXIT_FAILURE);
        }
    }

    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    //??? need search
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    std::cout << "Calculating normals for scale..." << scale1 << std::endl;

    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch (scale1);
    ne.compute (*normals_small_scale);
    // calculate normals with the large scale

    std::cout << "Calculating normals for scale..." << scale2 << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);
    ne.setRadiusSearch (scale2);
    ne.compute (*normals_large_scale);

    // Create output cloud for DoN results
    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>); //jai rajouté Ptr devant le new ... >::
    copyPointCloud (*cloud, *doncloud);

    std::cout << "Calculating DoN... " << std::endl;
      // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::PointNormal> don;
    don.setInputCloud(cloud);
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);

    if (!don.initCompute ())
    {
        std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
        exit (EXIT_FAILURE);
    }

    // Compute DoN
    don.computeFeature (*doncloud);

    // Save DoN features
    pcl::PCDWriter writer;
    writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false);

    // Filter by magnitude
    std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;

    // Build the condition for filtering
    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (
    new pcl::ConditionOr<pcl::PointNormal> ()
    );
    range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
                               new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                             );
    // Build the filter
    pcl::ConditionalRemoval<pcl::PointNormal> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (doncloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>); //jai rajouté Ptr devant le new ... >::

    // Apply filter
    condrem.filter (*doncloud_filtered);

    doncloud = doncloud_filtered;

    // Save filtered output
    std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

    writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false);

    // Filter by magnitude
    std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << angle << "..." << std::endl;

    pcl::search::KdTree<pcl::PointNormal>::Ptr segtree (new pcl::search::KdTree<pcl::PointNormal>);
    segtree->setInputCloud (doncloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;

    ec.setClusterTolerance (angle);
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (100000);
    ec.setSearchMethod (segtree);
    ec.setInputCloud (doncloud);
    ec.extract (cluster_indices);

    //TO DO
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<pcl::PointNormal>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster_don->points.push_back (doncloud->points[*pit]);
        }

        cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
        cloud_cluster_don->height = 1;
        cloud_cluster_don->is_dense = true;

        //Save cluster
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "don_cluster_" << j << ".pcd";
        writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);
    }
}

/**********************
    FIN MODELISATION
**********************/
/*********************************
*
*			SLOTS
*
*********************************/

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//affichage du nuage de point original
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

void MainWindow::chooseViewCloud(){
	this->view_plan = false;
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
// affichage du plan après les différents traitements/calculs/modélisations
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

void MainWindow::chooseViewPlane(){
	this->view_plan = true;
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//changement de la valeur du slider threshold
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

void MainWindow::changeThreshold(int th){
	this->threshold = th * 1.0f;
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//changement de la valeur du slider proba 
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

void MainWindow::changeProba(int proba){
	this->proba = proba / 100.0f;
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//sélection du fichier ply pour le traitement
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

void MainWindow::chooseFile(){
	this->file = QFileDialog::getOpenFileName(this,
	tr("Ouvrir fichier"), "",
	tr("Polygon File Format(*.ply);;"
	"Tous les fichiers(*)"));
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//fonction mère , appelle les autres fonctions et gère l'affichage
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

void MainWindow::draw(){
	/* initialize random seed: */
	srand (time(NULL));

	qDebug() << "draw";
	std::cout << "proba " << this->proba << std::endl;
	std::cout << "threshold " << this->threshold << std::endl;

	// initialize PointClouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::visualization::PCLVisualizer::Ptr viewer;
	//-------------load cloud
	std::vector<int> filenames;
	bool file_is_pcd = false;
	
	if (this->file == ""){
		std::cout << "Aucun fichier choisi." << std::endl;
		return;
	}
	if (file_is_pcd) { //not use
		if (pcl::io::loadPCDFile (this->file.toStdString(), *cloud) < 0)  {
			qDebug() << "Error loading point cloud " << this->file;
			return;
		}
	} else {
		if (pcl::io::loadPLYFile (this->file.toStdString(), *cloud) < 0)  {
			qDebug() << "Error loading point cloud " << this->file;
			return;
		}
	}
	//-- end load cloud

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    copyPointCloud(*cloud,*cloud_xyzrgb); //initialisation des points avec les couleur se passe correctement ? apparement non donc il va falloir set les couleurs des points à 0
    cout<<"nb_points : "<<cloud_xyzrgb->points.size()<<endl;
    don_segmentation(cloud_xyzrgb, 3.14, 0.5,0, 1); //always in the range (0,1)
    this->viewer = simpleVis(cloud);
	//cout << "nb plan :" << this->nb_cloud << endl;

	//finaliseVis(viewer);
    while (!this->viewer->wasStopped ())
	{
        this->viewer->spinOnce (100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

}

/*********************************
*
*   Transformation de nuage
*
*********************************/
/*void MainWindow::rotateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int degrees, int axe){
    QVector<float> mat_rotation_tmp;
    cout << "m_pi" << M_PI << endl;
    float value_rad = degrees * M_PI /180;
    cout << "value_rad" << value_rad << endl;

    if(axe == 0) {
        cout << "vtf1" << endl;
        mat_rotation_tmp.push_back(1.0f);
        mat_rotation_tmp.push_back(0.0f);
        mat_rotation_tmp.push_back(0.0f);

        mat_rotation_tmp.push_back(0.0f);
        mat_rotation_tmp.push_back(cos(value_rad));
        mat_rotation_tmp.push_back(-sin(value_rad));

        mat_rotation_tmp.push_back(0.0f);
        mat_rotation_tmp.push_back(sin(value_rad));
        mat_rotation_tmp.push_back(cos(value_rad));
    } else if(axe == 1) {
        mat_rotation_tmp.push_back(cos(value_rad));
        mat_rotation_tmp.push_back(0.0f);
        mat_rotation_tmp.push_back(sin(value_rad));

        mat_rotation_tmp.push_back(0.0f);
        mat_rotation_tmp.push_back(1.0f);
        mat_rotation_tmp.push_back(0.0f);

        mat_rotation_tmp.push_back(-sin(value_rad));
        mat_rotation_tmp.push_back(0.0f);
        mat_rotation_tmp.push_back(cos(value_rad));
    } else {
        mat_rotation_tmp.push_back(cos(value_rad));
        mat_rotation_tmp.push_back(-sin(value_rad));
        mat_rotation_tmp.push_back(0.0f);

        mat_rotation_tmp.push_back(sin(value_rad));
        mat_rotation_tmp.push_back(cos(value_rad));
        mat_rotation_tmp.push_back(0.0f);

        mat_rotation_tmp.push_back(0.0f);
        mat_rotation_tmp.push_back(0.0f);
        mat_rotation_tmp.push_back(1.0f);
    }

    matrix mat_rotation(3, 3, mat_rotation_tmp);
    for (size_t i=0 ; i< cloud->points.size() ; i++){
        cout << "vtf" << endl;
        QVector<float> tmp;
        tmp.push_back(cloud->points[i].x);
        tmp.push_back(cloud->points[i].y);
        tmp.push_back(cloud->points[i].z);
        matrix point(3, 1, tmp);

        cout << "truc" << endl;
        //cout << "point_tmp" << point_tmp<< endl;
        //cout << "mat_rotation" << mat_rotation<< endl;
        //point.afficher();
        point = point.multMat(mat_rotation);

    cout << "muche" << endl;

        cloud->points[i].x = point.at(0, 0);
        cloud->points[i].y = point.at(0, 1);
        cloud->points[i].z = point.at(0, 2);
        cout << "good" << endl;
    }
}*/
//http://pointclouds.org/documentation/tutorials/matrix_transform.php
pcl::PointCloud<pcl::PointXYZ>::Ptr MainWindow::rotateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int degrees, int axe){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    float theta = degrees * M_PI /180; //passage des degrees en radians

    //création de la transformation en fonction de l'axe choisi
    if(axe == 0) //axe x
        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
    else if (axe == 1)
        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
    else
        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    // exécution de la transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // application de la transformation au nuage initial
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
    
/*********************************
*
*	Repérage des murs/sol/plafond
*
*********************************/
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//fonction mère , appelle les autres fonctions et gère l'affichage
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
void MainWindow::repereRoom(pcl::visualization::PCLVisualizer::Ptr viewer,std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> list_planes){
    cout<< "Repere room" << endl;
    std::vector<QVector3D> list_positions_mean;
    QVector3D pos_min = QVector3D(FLT_MAX, FLT_MAX, FLT_MAX);
    QVector3D pos_max = QVector3D(FLT_MIN, FLT_MIN, FLT_MIN);

    cout<< "nombre plan total" << list_planes.size() << endl;
    for (unsigned i = 0; i < list_planes.size(); i++){
        int cpt = 0;
        QVector3D pos_mean_plane = QVector3D(0,0,0);
        //calcul de la position moyenne de chaque plans
        cout<< "list_planes[i]->points.size()" << list_planes[i]->points.size() << endl;
        for (size_t j=0;j< list_planes[i]->points.size();j++){
            pos_mean_plane[0] += list_planes[i]->points[j].x;
            //cout<< "pos_mean_plane[0]" << pos_mean_plane[0]<< endl; //nan
            //cout<< "list_planes[i]->points[j].x" << list_planes[i]->points[j].x << endl; //nan

            pos_mean_plane[1] += list_planes[i]->points[j].y;
            pos_mean_plane[2] += list_planes[i]->points[j].z;
            cpt++;
        }
        pos_mean_plane[0] /= cpt;
        pos_mean_plane[1] /= cpt;
        pos_mean_plane[2] /= cpt;
        list_positions_mean.push_back(pos_mean_plane);
        cout<< "pos_mean_plane" << pos_mean_plane.y()<< endl; //nan


        //recherche du plan le plus bas et le plus haut
        if(pos_mean_plane[1] > pos_max[1])
            pos_max = pos_mean_plane;
        if(pos_mean_plane[1] < pos_min[1])
            pos_min = pos_mean_plane;

    }
    //------reconstitution du sol et plafond grace a un seuil d'erreur
    float error = (pos_max[1] - pos_min[1]) * 0.05; //(5% de la hauteur de la piece)
    cout<< "pos min" << pos_min[1] << endl;
    cout<< "pos max" << pos_max[1] << endl;
    cout<< "error" << error << endl;

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> list_floor; //liste des plans appartenant au sol
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> list_ceiling;//liste des plans appartenant au plafond
    for (unsigned i = 0; i < list_positions_mean.size(); i++){
        //sol
        cout<< "list_positions_mean[" <<i<<"][1]" << list_positions_mean[i].y()<< endl; //nan
        cout<< "pos min" << pos_min.y() << endl;
        cout<< "(pos_min[1]+error)" << (pos_min.y()+error) << endl;
        if (list_positions_mean[i].y() >= pos_min.y() && list_positions_mean[i].y() <= (pos_min.y()+error)){
            list_floor.push_back(list_planes[i]);
        }
        //plafond
        if (list_positions_mean[i].y() <= pos_max.y() && list_positions_mean[i].y() >= (pos_max.y()+error)){
            list_ceiling.push_back(list_planes[i]);
        }
    }

    cout<< "nombre plan floor" << list_floor.size() << endl;
    cout<< "nombre plan ceiling" << list_ceiling.size() << endl;

    //ajout au viewer des élements dans des couleurs différentes
    for (unsigned i = 0; i < list_floor.size(); i++){
        for(size_t j=0; j < list_floor[i]->size(); j++){
            list_floor[i]->points[j].r = 255;
            list_floor[i]->points[j].g = 0;
            list_floor[i]->points[j].b = 0;
        }
        addPtsCloudXYZRGB(viewer, list_floor[i]);
    }
    for (unsigned i = 0; i < list_ceiling.size(); i++){
        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color (list_ceiling[i], 0, 0, 255);
        for(size_t j=0; j < list_ceiling[i]->size(); j++){
            list_ceiling[i]->points[j].r = 0;
            list_ceiling[i]->points[j].g = 0;
            list_ceiling[i]->points[j].b = 255;
        }
        addPtsCloudXYZRGB(viewer, list_ceiling[i]);
    }
}

pcl::visualization::PCLVisualizer::Ptr MainWindow::addPtsCloudXYZRGB (pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // -----------------------------------------------
    // -----Open 3D viewer and add color point cloud--
    // -----------------------------------------------

    cout<<"ADD COLOR POINTCLOUD N°"<<this->nb_cloud<<endl;
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud"+this->nb_cloud);
    this->nb_cloud = this->nb_cloud +1;
    return (viewer);
}


