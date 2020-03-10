//Create with the help of http://pointclouds.org/documentation/tutorials/random_sample_consensus.php

#include "mainwindow.h"

#include "glarea.h"

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
    connect(ui->slider_threshold, SIGNAL(valueChanged(int)), this, SLOT(changeThreshold(int))); //connexion slidebar threshold
    connect(ui->slider_proba, SIGNAL(valueChanged(int)), this, SLOT(changeProba(int))); //connexion slidebar proba
    connect(ui->btn_import, SIGNAL(clicked()), this, SLOT(chooseFile())); //connexion selection de fichier
    connect(ui->btn_displayBasic, SIGNAL(clicked()), this, SLOT(displayBasic())); //connexion bouton draw, on lance le viewer
    connect(ui->btn_draw, SIGNAL(clicked()), this, SLOT(draw())); //connexion bouton draw, on lance le viewer
    connect(ui->btn_segmente, SIGNAL(clicked()), this, SLOT(segmente())); //connexion bouton
    connect(ui->btn_modelize, SIGNAL(clicked()), this, SLOT(modelize())); //connexion bouton modelize, on lance la modelisation

    /*processViewer = new QThread();
    this->moveToThread(processViewer);
    processViewer->start();*/
	//save
    //connect(ui->btn_save, SIGNAL(clicked()), this, SLOT(saveCloud()));

}

MainWindow::~MainWindow()
{
	delete ui;
}

/**********************************
*
*          PCL (avant la partie PFE, les fonctions ont été récupérées du projet de TER 2018/2019 en M1 de CLAUDET-GLAISE-PEPINO)
*
**********************************/

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//SLOT - sauvegarde du nuage de point dans un fichier ply
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

void MainWindow::saveCloud()
{
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

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//TODO
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
void MainWindow::showVizualizer(){

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
    cout<<"ADD POINTCLOUD N°"<<this->nb_cloud<<endl;
	viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud"+this->nb_cloud);
	this->nb_cloud = this->nb_cloud +1;
    cout << "affichage fait" << endl;
	return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr MainWindow::addPtsCloud (pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
	// -----------------------------------------------
	// -----Open 3D viewer and add color point cloud--
	// -----------------------------------------------
    cout<<"ADD NORMAL POINTCLOUD N°"<<this->nb_cloud<<endl;
    //viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud"+this->nb_cloud);
	this->nb_cloud = this->nb_cloud +1;
	return (viewer);
}
/**********************
      DEBRUITAGE
**********************/
void MainWindow::denoise(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
    std::cerr << "denoising running " << std::endl;

    //creer nuage inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
    // creer nuage outliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // creer cloud pour futur nuage filtré
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (10); //nombre de plus proche voisins
    sor.setStddevMulThresh (0.3);

    sor.filter (*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    // les données restantes (inliers) sont ecrites sur le disque pour une utilisation ulterieure
    pcl::PLYWriter writer;
    writer.write<pcl::PointXYZ> ("../clouds/test_noise_inliers.ply", *cloud_filtered, false);

    std::cerr << "Cloud filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    std::cerr << "Cloud outliers: " << std::endl;
    std::cerr << *cloud_outliers << std::endl;

    std::cerr << "denoising end " << std::endl;
}
    /*------------------------------------------------ PFE ------------------------------------------------*/

/**********************
   RECHERCHE DE PLAN
**********************/
/**
 * @brief MainWindow::don_segmentation - Segmentation par direction des normales et agrandissement des régions
 * @param cloud - nuage initial
 * @param angle TODO
 * @param threshold TODO
 * @param scale1 TODO
 * @param scale2 TODO
 * @note n'est pas utilisé
 */
void MainWindow::don_segmentation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double angle, double threshold, double scale1, double scale2)
{
    ///The smallest scale to use in the DoN filter.
    //scale1 = 0.2;

    ///The largest scale to use in the DoN filter.
    //scale2 = 2.0;

    ///The minimum DoN magnitude to threshold by
    //threshold = 0.25;

    ///segment scene into clusters with given distance tolerance using euclidean clustering
    double segradius = angle;

    //voxelization factor of pointcloud to use in approximation of normals
    bool approx = false; //FALSE
    constexpr double decimation = 100; //100

    pcl::search::Search<pcl::PointXYZ>::Ptr tree;

    if (cloud->isOrganized ())
    {
        tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
    }
    else
    {
      tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
    }

    tree->setInputCloud (cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr small_cloud_downsampled;
    pcl::PointCloud<pcl::PointXYZ>::Ptr large_cloud_downsampled;

    // If we are using approximation
    if(approx)
    {
        std::cout << "Downsampling point cloud for approximation" << std::endl;

        // Create the downsampling filtering object
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setDownsampleAllData (false);
        sor.setInputCloud (cloud);

        // Create downsampled point cloud for DoN NN search with small scale
        small_cloud_downsampled = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        float smalldownsample = static_cast<float> (scale1 / decimation);
        sor.setLeafSize (smalldownsample, smalldownsample, smalldownsample);
        sor.filter (*small_cloud_downsampled);
        std::cout << "Using leaf size of " << smalldownsample << " for small scale, " << small_cloud_downsampled->size() << " points" << std::endl;

        // Create downsampled point cloud for DoN NN search with large scale
        large_cloud_downsampled = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        const float largedownsample = float (scale2/decimation);
        sor.setLeafSize (largedownsample, largedownsample, largedownsample);
        sor.filter (*large_cloud_downsampled);
        std::cout << "Using leaf size of " << largedownsample << " for large scale, " << large_cloud_downsampled->size() << " points" << std::endl;

        //this->viewer = addPtsCloudXYZ (viewer, large_cloud_downsampled); //ADDVIEWER
    }

    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud (cloud);
        ne.setSearchMethod (tree);

    /**
    * NOTE: setting viewpoint is very important, so that we can ensure
    * normals are all pointed in the same direction!
    */
    ne.setViewPoint(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());


    if(scale1 >= scale2)
    {
        std::cerr << "Error: Large scale must be > small scale!" << std::endl;
        exit(EXIT_FAILURE);
    }

    //the normals calculated with the small scale
    std::cout << "Calculating normals for scale..." << scale1 << std::endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

    if(approx)
    {
        ne.setSearchSurface(small_cloud_downsampled);
    }

    ne.setRadiusSearch (scale1);
    ne.compute (*normals_small_scale);
    viewer = addPtsCloud(viewer, normals_small_scale); //ADD VIEWER
    std::cout << "Calculating normals for scale..." << scale2 << std::endl;
    //the normals calculated with the large scale
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

    if(approx)
    {
        ne.setSearchSurface(large_cloud_downsampled);
    }
    ne.setRadiusSearch (scale2);
    ne.compute (*normals_large_scale);

    viewer = addPtsCloud(viewer, normals_large_scale); //ADD VIEWER
    // Create output cloud for DoN results
    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
    copyPointCloud (*cloud, *doncloud);

    std::cout << "Calculating DoN... " << std::endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
    don.setInputCloud (cloud);
    don.setNormalScaleLarge(normals_large_scale);
    don.setNormalScaleSmall(normals_small_scale);

    if(!don.initCompute ())
    {
        std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
        exit(EXIT_FAILURE);
    }

    //Compute DoN
    don.computeFeature(*doncloud);
    std::cout << "Pointcloud DON NBPOINT : " << doncloud->points.size () << " data points." << std::endl;
    pcl::PLYWriter writer;

    //Save DoN features
    std::string outfile = "doncloud.ply";
    writer.write<pcl::PointNormal> (outfile, *doncloud, false);

    //Filter by magnitude
    std::cout << "Filtering out DoN mag <= "<< threshold <<  "..." << std::endl;

    // build the condition
    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (new
    pcl::ConditionOr<pcl::PointNormal> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
    pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointNormal> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (doncloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

    // apply filter
    condrem.filter (*doncloud_filtered);

    //doncloud = doncloud_filtered; //filtered

    // Save filtered output
    std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;
    std::stringstream ss;
    if(doncloud->points.size ()<=0 )
    {
        cout<<"RETURN ERROR STATEMENT"<<endl;
        return;
    }
    else
    {
        cout<<"SUCCES : CLOUDSIZE > 0 "<<endl;
    }
    ss << outfile.substr(0,outfile.length()-4) << "_threshold_"<< threshold << "_.ply";
    writer.write<pcl::PointNormal> (ss.str (), *doncloud, false);

    //Filter by magnitude
    std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= "<< segradius <<  "..." << std::endl;

    pcl::search::KdTree<pcl::PointNormal>::Ptr segtree (new pcl::search::KdTree<pcl::PointNormal>);
    segtree->setInputCloud (doncloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
    ec.setClusterTolerance (segradius);
    ec.setMinClusterSize (75); //50
    ec.setMaxClusterSize (10000000); //100000
    ec.setSearchMethod (segtree);
    ec.setInputCloud (doncloud);
    ec.extract (cluster_indices);

    int j = 0;
    cout<<"Number of CLUSTERS : "<<cluster_indices.size()<<endl;
    if(cluster_indices.size()<=0)
    {
        cout<<"CLUSTER IS EMPTY"<<endl;
        cout<<"UNSUCCES WAY"<<endl;
    }

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_don_color (new pcl::PointCloud<pcl::PointXYZ>);
        for (const int &index : it->indices)
        {
            cloud_cluster_don->points.push_back (doncloud->points[index]);
            cloud_cluster_don_color->points.push_back(cloud->points[index]);
            cout<<cloud->points[index].x<<endl;

        }
        this->vector_cloud_RGB.push_back(*cloud_cluster_don_color);
        cout<<"CLUSTER SIZE "<<cloud_cluster_don_color->points.size()<<endl;
        //this->viewer = addPtsCloudXYZRGB (viewer, cloud_cluster_don_color);
        cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
        cloud_cluster_don->height = 1;
        cloud_cluster_don->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << outfile.substr(0,outfile.length()-4) << "_threshold_"<< threshold << "_cluster_" << j << ".ply";
        //writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false); //save in a file
    }
    //cout<<"j = "<<j<<endl;
    if(j>0)
    {
        cout<<"SUCCES : NB_CLUSTER > 0"<<endl;
    }
}

/**
 * @brief MainWindow::ransac_segmentation - recherche tous les plans présents dans le nuage initial cloud à l'aide de l'algorithme de RANSAC
 */
void MainWindow::ransac_segmentation()
{
    //debruitage
    // denoise(cloud); //on enlève le bruit du nuage de points
    //  //le minimum de points que doit comporter un nuage de points pour être conservé
    // pcl::io::loadPLYFile ("./test_noise_inliers.ply", *cloud); //récupération du fichier dans lequel il y a le nuage de point débruité
    //mettre le nuage de point en couleur et creer un viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 0, 255); //initialisation d'une couleur bleue pour les points
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cpy = cloud;


    //int maxToDelete = 0.02*cloud->size(); //attention lors du merge

    int j = 0;

    while(cloud->size()>300) //on récupère tout les plans tant qu'il reste plus de 100 pts dans le nuage de point original
    {
        j++;
        //std::cout<<cloud->size()<<endl; //affichage de la taille du nuage
        pcl::ExtractIndices<pcl::PointXYZ> extract; //vecteur qui contiendra les points des plans à extraire du nuage original
        std::vector<int> inliers; //vecteur qui contiendra les points des plans
        pcl::PointIndices::Ptr suppression_inliers(new pcl::PointIndices()); //necessaire pour la suppression des points du nuage original

        // created RandomSampleConsensus object and compute the appropriated model
        // ======= RANSAC ============
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
        model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud)); //création d'un modèle pour la récupération de points formant un plan

        std::cout << "Appel de ransac" << endl;
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
        ransac.setDistanceThreshold (this->threshold);
        ransac.setProbability(this->proba);
        ransac.computeModel();
        ransac.getInliers(inliers);
        // ============ FIN RANSAC ====================

        // copies all inliers of the model computed to another PointCloud
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

        for(int i = 0; i < inliers.size(); i++)
        {
            suppression_inliers->indices.push_back(inliers.at(i)); // mettre notre inliers dans un autre inlier (suppr_inlier) (->les 2 inliers ne sont pas de meme type et on a besoin des 2 pour nos fonctions)
        }

        //récupération des équations des plans
        //if(final->size() > maxToDelete) //si la taille du plan est suppérieure à un pourcentage de la taille du nuage de point global donné
        //{

            std::vector<float> list_coeff; //liste des coefs de l'équation du plan
            std::vector<float> sum; //vector d'addition des coefs pour en faire une moyenne
            for(int i = 0; i < 4; ++i)
                sum.push_back(0.0); //init sum

            int nb_iter_for_avg = 1; //nombre de fois qu'on itere pour faire la moyenne des equations des plans

            for(int i= 0; i < nb_iter_for_avg; ++i)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d (final); //  creer nuage (copie de final)
                //Eigen::VectorXf ground_coeffs;
                //ground_coeffs.resize(4); // coefficient du plan
                std::vector<int> clicked_points_indices; //indices des points du plan  choisis random
            }
            //calcul de la moyenne pour chaque valeur de l equation
            int iter_eq = 4;
            for(int i = 0; i < iter_eq; ++i)
                list_coeff.push_back(sum[i]);
                //list_coeff.push_back(sum[i]/nb_iter_for_avg);

            pcl::ModelCoefficients plane_coeff;
            plane_coeff.values.resize (4);
            plane_coeff.values[0] = list_coeff[0];
            plane_coeff.values[1] = list_coeff[1];
            plane_coeff.values[2] = list_coeff[2];
            plane_coeff.values[3] = list_coeff[3];

            vector_eq.push_back(plane_coeff); //on stocke l'équation dans un vecteur pour l'étape dite du maillage

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            single_color_gen (final, (j*100)%255, (j*20)%255, (j*30)%255); //la couleur de chaque nuage de plan ransac est différente

            addPtsCloudColor(viewer,final,single_color_gen); //ajout du nuage de plan ransac au viewer
			pcl::PointCloud<pcl::PointXYZ> cpy;
			copyPointCloud(*final, cpy);
			vector_cloud.push_back(cpy);
            cout << "Taille vector_cloud : " << vector_cloud.size();
        //}


        //ici on supprime du nuage de point original le plan trouvé par ransac
        // comme ça lors de la prochaine itération, ransac sélectionne un autre plan
        extract.setInputCloud(cloud);
        extract.setIndices(suppression_inliers);
        extract.setNegative(true);
        extract.filter(*cloud);

        have_plane = true; //il y a au moins un plan trouvé par ransac
    }
}

/**********************
    EQUATION DE PLAN
**********************/
/**
 * @brief MainWindow::equation_plane - calcul l'équation du plan passant par les 3 points
 * @param P1
 * @param P2
 * @param P3
 * @return un tableau de double représentant les a, b, c et d d'une équation de plan
 */
double * MainWindow::equation_plane(QVector3D P1, QVector3D P2, QVector3D P3)
{
    double * eq_plane = new double[4];

    double a = static_cast<double>((P2[1]-P1[1])*(P3[2] - P1[2]) - (P2[2]-P1[2])*(P3[1]-P1[2]));
    double b = static_cast<double>(-((P2[0]-P1[0])*(P3[2]-P1[2])-(P2[2]-P1[2])*(P3[0]-P1[0])));
    double c = static_cast<double>((P2[0]-P1[0])*(P3[1]-P1[1]) - (P2[1]-P1[1])*(P3[0]-P1[0]));
    double d = static_cast<double>(-( a * P1[0] + b * P1[1] + c * P1[2]));
    //qDebug() << "-( a * P1[0] + b * P1[1] + c * P1[2]) :" << a << "*" << P1[0] << "+" << b << "*" << P1[1] << "+" << c << "*" << P1[2];
    //qDebug() << "equation of plane is " << a << " x + " << b
        //<< " y + " << c << " z + " << d << " = 0.";
    eq_plane[0]=a;
    eq_plane[1]=b;
    eq_plane[2]=c;
    eq_plane[3]=d;

    return eq_plane;
}
/**
 * @brief MainWindow::equation_plane2 - retrouve une approximation de l'équation du plan en utilisant RANSAC
 * @param pcloud - nuage de points représentant un plan
 * @return un tableau de double représentant les a, b, c et d d'une équation de plan
 */
double * MainWindow::equation_plane2(pcl::PointCloud<pcl::PointXYZ> pcloud)
{
    double * myEq = new double[4];
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.90f); //CHANGE
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcloudptr(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(pcloud,*pcloudptr);
    seg.setInputCloud (pcloudptr);
    seg.segment (*inliers, *coefficients);

    std::cerr << "Model coefficients PLANE2: " 
    		  << coefficients->values[0] << " x + "
    		  << coefficients->values[1] << " y + "
              << coefficients->values[2] << " z + "
              << coefficients->values[3] << std::endl;

    myEq[0] = coefficients->values[0];
    myEq[1] = coefficients->values[1];
    myEq[2] = coefficients->values[2];
    myEq[3] = coefficients->values[3];
    return myEq;
}

/**
 * @brief MainWindow::moy_eq_plane On prend plusieurs trinômes de points des nuages afin de faire une équation de plan moyenne
 * @param pcloud - nuage de points représentant un plan
 * @return une équation de plan moyenne
 */
double *MainWindow::moy_eq_plane(pcl::PointCloud<pcl::PointXYZ> pcloud)
{
    std::vector<double *> vec_eq;
    for(int i = 0; i<pcloud.points.size()-3;i++)
    {
        QVector3D p1(pcloud.points[i].x,pcloud.points[i].y,pcloud.points[i].z);
        QVector3D p2(pcloud.points[i+1].x,pcloud.points[i+1].y,pcloud.points[i+1].z);
        QVector3D p3(pcloud.points[i+2].x,pcloud.points[i+2].y,pcloud.points[i+2].z);

        vec_eq.push_back(equation_plane(p1,p2,p3));
    }
    double * eq_final = new double[4];
    for(int i = 0; i< vec_eq.size(); i++)
    {
        eq_final[0] = eq_final[0] + vec_eq.at(i)[0];
        eq_final[1] = eq_final[1] + vec_eq.at(i)[1];
        eq_final[2] = eq_final[2] + vec_eq.at(i)[2];
        eq_final[3] = eq_final[3] + vec_eq.at(i)[3];
    }
    eq_final[0] = eq_final[0]/vec_eq.size();
    eq_final[1] = eq_final[1]/vec_eq.size();
    eq_final[2] = eq_final[2]/vec_eq.size();
    eq_final[3] = eq_final[3]/vec_eq.size();
		cout<<"MOYENNE EQ PLANE : "<<eq_final[0]<<"x + "<<
    eq_final[1]<<"y + "<<
    eq_final[2]<<"z + "<<
    eq_final[3]<<" = 0 "<<endl;
    /*int a = rand()%(vec_eq.size()-1);
    eq_final[0] = vec_eq.at(a)[0];
    eq_final[1] = vec_eq.at(a)[1];
    eq_final[2] = vec_eq.at(a)[2];
    eq_final[3] = vec_eq.at(a)[3];*/
    return eq_final;
}

/**
 * @brief MainWindow::resol_3eq_3inc - résolution d'un système de 3 équations 3 inconnus
 * @param eq1 - tableau représentant a, b, c et d d'une équation
 * @param eq2 - tableau représentant a, b, c et d d'une équation
 * @param eq3 - tableau représentant a, b, c et d d'une équation
 * @return la solution de la résolution sous forme d'un tableau de 3 éléments avec x, y et z
 */
QVector3D MainWindow::resol_3eq_3inc(double * eq1, double * eq2, double * eq3)
{
    double matrice[3][4];
    double coefficient,x,y,z;
    int i=0;
    //qDebug() << "Resolution d'un systeme de 3 equations a trois inconnues\n";
    //qDebug() << "Premiere equation, entrez en ordre respectif x,y,z et la constante\n";
    //cin >> matrice[0][0]>>matrice[0][1]>>matrice[0][2]>>matrice[0][3];
    matrice[0][0] = eq1[0];
    matrice[0][1] = eq1[1];
    matrice[0][2] = eq1[2];
    matrice[0][3] = eq1[3];
    //cout << "Seconde equation, entrez en ordre respectif x,y,z et la constante\n";
    //cin >> matrice[1][0]>>matrice[1][1]>>matrice[1][2]>>matrice[1][3];
    matrice[1][0] = eq2[0];
    matrice[1][1] = eq2[1];
    matrice[1][2] = eq2[2];
    matrice[1][3] = eq2[3];
    //cout << "Premiere equation, entrez en ordre respectif x,y,z et la constante\n";
    //cin >> matrice[2][0]>>matrice[2][1]>>matrice[2][2]>>matrice[2][3];
    matrice[2][0] = eq3[0];
    matrice[2][1] = eq3[1];
    matrice[2][2] = eq3[2];
    matrice[2][3] = eq3[3];
    coefficient=(-1.0*matrice[1][0]/matrice[0][0]);

    /*for(int p = 0; p<3;p++)
        for(int l = 0; l<4; l++)
            qDebug() <<"matrice : "<<matrice[p][l]<<endl;*/

    for(;i<=3;i++)
    {
        matrice[1][i]=(coefficient*matrice[0][i])+matrice[1][i];
    }
    coefficient=(-1.0*matrice[2][0]/matrice[0][0]);
    i=0;
    for(;i<=3;i++)
    {
        matrice[2][i]=(coefficient*matrice[0][i])+matrice[2][i];
    }
    coefficient=(-1.0*matrice[2][1]/matrice[1][1]);
    i=1;
    for(;i<=3;i++)
    {
        matrice[2][i]=(coefficient*matrice[1][i])+matrice[2][i];
    }
    z=matrice[2][3]/matrice[2][2];
    y=(matrice[1][3]-(matrice[1][2]*z))/matrice[1][1];
    x=(matrice[0][3]-((matrice[0][1]*y)+(matrice[0][2]*z)))/matrice[0][0];

		x = -x;
		y = -y;
		z = -z;

    //qDebug()  << "X est egal a " << x << "\n";
    //qDebug()  << "Y est egal a " << y << "\n";
    //qDebug()  << "Z est egal a " << z << "\n";
    QVector3D point(x,y,z);
    //system("PAUSE");
    return point;
}

/**********************
       RECHERCHE DE LIMITES DE LA PIECE
**********************/
/**
 * @brief MainWindow::repereRoom - repérage des murs, sol, plafond d'une piece
 * @param viewer
 * @param list_planes - liste des plans retrouvés par la méthode de recherche de plan
 */
void MainWindow::repereRoom(pcl::visualization::PCLVisualizer::Ptr viewer, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list_planes){
    cout<< "Repere room" << endl;
    std::vector<QVector3D> list_meanPosition;
    std::vector<QVector3D> list_normale_planes;

    cout<< "nombre plan total" << list_planes.size() << endl;
    //initialise les vector comprenant la position moyenne/les équations/les normales de tous les plans pour eviter de recalculer plusieurs fois
    for (unsigned i = 0; i < list_planes.size(); i++)
    {
        list_meanPosition.push_back(computeMeanPositionPlane(list_planes[i]));
        list_normale_planes.push_back(QVector3D(eq_planes[i][0], eq_planes[i][1], eq_planes[i][2]));
    }
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> copy = list_planes;

    searchLimit(copy, eq_planes, list_normale_planes, list_meanPosition);

    createRoom();

    cout << "room.size() " << room.size() << endl;
    for(unsigned i = 0; i < room.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(room[i], *tmp);

        int r = 0;
        int g = 0;
        int b = 0;

        if (i == 0 || i == 1){
            b = 255;
        } else if (i == 2){
            g = 255;
        } else if (i == 3){
            b = 155;
            r = 155;
        } else {
            r = 255;
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        single_color_gen (tmp, r, g, b); //la couleur de chaque nuage de plan ransac est différente

        addPtsCloudColor (viewer, tmp, single_color_gen);
    }
}

//http://pointclouds.org/documentation/tutorials/matrix_transform.php
/**
 * @brief MainWindow::rotateCloud - fais pivoter un nuage de points en recalculant toutes les coordonnées
 * @param cloud - nuage a tourner
 * @param degrees - nombre de degrees voulu
 * @param axe - axe sur lequel faire pivoter (x = 0, y = 1, z = 2)
 * @return un nouveau nuage qui est une copie de l'ancien mais qui a pivoté
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr MainWindow::rotateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int degrees, int axe){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    float theta = degrees * M_PI /180; //passage des degrees en radians

    //création de la transformation en fonction de l'axe choisi
    if(axe == 0) //axe x
        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
    else if (axe == 1) //axe y
        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
    else
        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    // exécution de la transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // application de la transformation au nuage initial
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
    return transformed_cloud;
}

/**
 * @brief MainWindow::searchLimit - recherche les éléments les plus éloignés dans la pièce pour retrouver ce qui semble etre sol/plafond/mur
 * @param list_planes - liste des plans retrouvés par la méthode de recherche de plan
 * @param list_equat_planes - tableau des équations de chaque plan
 * @param list_normale_planes - tableau des normales de chaque plan
 * @param list_meanPosition - tableau des positions moyennes de chaque plan
 */
void MainWindow::searchLimit (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list_planes, std::vector<double *> list_equat_planes, std::vector<QVector3D> list_normale_planes, std::vector<QVector3D> list_meanPosition){

    cout << "searchLimit" << endl;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planes_axe_x;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planes_axe_y;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planes_axe_z;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planes_others;
    cout << "taille list_planes : " << list_planes.size() << ", taille list_equat_planes : " << list_equat_planes.size() << endl;

    std::vector<int> corres_axe_x;
    std::vector<int> corres_axe_y;
    std::vector<int> corres_axe_z;

    QVector3D axe_x(1,0,0);
    QVector3D axe_y(0,1,0);
    QVector3D axe_z(0,0,1);

	//    1 - parcourir tous les plans
	//        2 - chercher les plans pour chaque axe (comparer les normales) trier en trois
	for(unsigned i = 0; i < list_planes.size(); i++){
	    if(normalesAreSimilar(axe_x, list_normale_planes[i])){
	        // cout << "je suis sur x" << endl;
	        planes_axe_x.push_back(list_planes[i]);
	        corres_axe_x.push_back(i);
	    } else if(normalesAreSimilar(axe_y, list_normale_planes[i])){
	        // cout << "je suis sur y" << endl;
	        planes_axe_y.push_back(list_planes[i]);
	        corres_axe_y.push_back(i);
	        // cout << "normale du plan : " << list_equat_planes[i][0] << ", " << list_equat_planes[i][1] << ", " << list_equat_planes[i][2] << endl;
	    } else if(normalesAreSimilar(axe_z, list_normale_planes[i])){
	        // cout << "je suis sur z" << endl;
	        planes_axe_z.push_back(list_planes[i]);
	        corres_axe_z.push_back(i);
	    } else {
	        // cout << "je suis sur rien" << endl;
	        planes_others.push_back(list_planes[i]);
	    }
	}

    // 3 - parcourir les 3 vectors
    //     4 - trouver position min max de chaque axe dans l'axe du vector

    float min_x = FLT_MAX;
    float min_y = FLT_MAX;
    float min_z = FLT_MAX;

    float max_x = FLT_MIN;
    float max_y = FLT_MIN;
    float max_z = FLT_MIN;

    //axe x (murs)
    for(unsigned i = 0; i < planes_axe_x.size(); i++){
        QVector3D valeur_courante = list_meanPosition[corres_axe_x[i]];
        if(valeur_courante[0] > max_x) max_x = valeur_courante[0];
        else if(valeur_courante[0] < min_x) min_x = valeur_courante[0];
    }
    //axe y (plafond/sol)
    for(unsigned i = 0; i < planes_axe_y.size(); i++){
        QVector3D valeur_courante = list_meanPosition[corres_axe_y[i]];
        if(valeur_courante[1] > max_y) max_y = valeur_courante[1];
        else if(valeur_courante[1] < min_y) min_y = valeur_courante[1];
    }
    //axe z (murs)
    for(unsigned i = 0; i < planes_axe_z.size(); i++){
        QVector3D valeur_courante = list_meanPosition[corres_axe_z[i]];
        if(valeur_courante[2] > max_z) max_z = valeur_courante[2];
        else if(valeur_courante[2] < min_z) min_z = valeur_courante[2];
    }


    // 5 - calculer taille piece en fonction des positions --> calculer 3 seuil erreur = 5%
    float seuil = 0.04f;
    float error_x = (max_x - min_x) * seuil;
    float error_y = (max_y - min_y) * seuil;
    float error_z = (max_z - min_z) * seuil;


    // 6 - parcourir les 3 vecteurs
    //     7 - ajouter les plans en fonctions du seuil d erreur a des fragments
    for(unsigned i = 0; i < 6; i++){
        list_limits.push_back(Fragment());
    }
    //axe x (murs) Fragment 1 et 2
    cout << "Il y a " << planes_axe_x.size() << " plans sur x, " << planes_axe_y.size() << " plans sur y, " << planes_axe_z.size() << " plans sur z." << endl;
    for(unsigned i = 0; i < planes_axe_x.size(); i++){
        QVector3D valeur_courante = list_meanPosition[corres_axe_x[i]];
        if(valeur_courante[0] > (max_x-error_x)  && valeur_courante[0] < (max_x+error_x)){
            list_limits[0].addPlane(*planes_axe_x[i]);
            // cout << "je suis un mur x 1 " << endl;
        }
        else if(valeur_courante[0] > (min_x-error_x)  && valeur_courante[0] < (min_x+error_x)){
            list_limits[1].addPlane(*planes_axe_x[i]);
            // cout << "je suis un mur x 2" << endl;
        }
    }
    //axe y (plafond/sol) Fragment 3 et 4
    for(unsigned i = 0; i < planes_axe_y.size(); i++){
        QVector3D valeur_courante = list_meanPosition[corres_axe_y[i]];
        if(valeur_courante[1] > (max_y-error_y)  && valeur_courante[1] < (max_y+error_y)){
            list_limits[2].addPlane(*planes_axe_y[i]);
            cout << "je suis un plafond d equation " << list_equat_planes[corres_axe_x[i]][0] << ", "  << list_equat_planes[corres_axe_x[i]][1]
                    << ", "  << list_equat_planes[corres_axe_x[i]][2] << endl;

        }
        else if(valeur_courante[1] > (min_y-error_y)  && valeur_courante[1] < (min_y+error_y)){
            list_limits[3].addPlane(*planes_axe_y[i]);
            // cout << "je suis un sol" << endl;
        }
    }
    //axe z (murs) Fragment 5 et 6
    for(unsigned i = 0; i < planes_axe_z.size(); i++){
        QVector3D valeur_courante = list_meanPosition[corres_axe_z[i]];
        if(valeur_courante[2] > (max_z-error_z)  && valeur_courante[2] < (max_z+error_z)){
            list_limits[4].addPlane(*planes_axe_z[i]);
            // cout << "je suis un mur z 1" << endl;
        }
        else if(valeur_courante[2] > (min_z-error_z)  && valeur_courante[2] < (min_z+error_z)){
            list_limits[5].addPlane(*planes_axe_z[i]);
            // cout << "je suis un mur z 2" << endl;
        }
    }

    //attribution type aux fragments
    list_limits[0].setType(Fragment::Type::Mur);
    list_limits[1].setType(Fragment::Type::Mur);

    list_limits[2].setType(Fragment::Type::Plafond);
    list_limits[3].setType(Fragment::Type::Sol);

    list_limits[4].setType(Fragment::Type::Mur);
    list_limits[5].setType(Fragment::Type::Mur);
    for(unsigned i = 0; i < list_limits.size(); i++){
        list_limits[i].attributeColorAuto();
    }
    cout << "Il y a " << list_limits[0].planes.size() << " mur en x1, " << list_limits[1].planes.size() << " mur en x2, " <<
            list_limits[2].planes.size() << " plafond, " <<list_limits[3].planes.size() << " sol, " <<
            list_limits[4].planes.size() << "mur en z1, " << list_limits[5].planes.size() << "mur en z2." << endl;
}

/**
 * @brief MainWindow::createRoom - réunis les différents plans formant un pan de la piece pour créer une piece avec un minimum de nuage (un nuage = un pan)
 */
void MainWindow::createRoom(){
    for(unsigned i = 0; i < list_limits.size(); i++){
        pcl::PointCloud<pcl::PointXYZ> tmp = clouds_union(list_limits[i].planes);
        room.push_back(tmp);
    }
}

/**
 * @brief MainWindow::clouds_union
 * @param v_cloud - tableau de nuages de points
 * @return un nuage qui est l'union de tous les nuages du vector
 */
pcl::PointCloud<pcl::PointXYZ> MainWindow::clouds_union(std::vector<pcl::PointCloud<pcl::PointXYZ>> v_cloud) {
    pcl::PointCloud<pcl::PointXYZ> c_union;
    for(int i = 0; i<v_cloud.size();i++)
    {
        c_union = c_union + v_cloud.at(i);
    }
    return c_union;
}

/*---- calcul ----*/
/**
 * @brief MainWindow::computeMeanPositionPlane - calcul la position moyenne des points d'un nuage
 * @param plane - le nuage de points initial (appelé plane dans notre contexte car n'est intéressant que pour nos nuages de plan)
 * @return un QVector3D représentant le x, y, et z moyen
 */
QVector3D MainWindow::computeMeanPositionPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr plane){
    int cpt = 0;
    QVector3D pos_mean_plane = QVector3D(0,0,0);
    //calcul de la position moyenne d'un plan
    for (size_t j=0;j< plane->points.size();j++){
        pos_mean_plane[0] += plane->points[j].x;
        pos_mean_plane[1] += plane->points[j].y;
        pos_mean_plane[2] += plane->points[j].z;
        cpt++;
    }
    pos_mean_plane[0] /= cpt;
    pos_mean_plane[1] /= cpt;
    pos_mean_plane[2] /= cpt;
    return pos_mean_plane;
}

/**
 * @brief MainWindow::computeDistance - calcul la distance entre deux points dans l'espace
 * @param p1 - QVector3D d'un point
 * @param p2 - QVector3D d'un point
 * @return une distance
 */
double MainWindow::computeDistance(QVector3D p1, QVector3D p2){
    return sqrt( pow(p1[0]-p2[0],2) + pow(p1[1]-p2[1],2) + pow(p1[2]-p2[2],2) );
}

/**
 * @brief MainWindow::normalesAreSimilar - vérifie si deux normales sont a peu pres semblables (ou inverses)
 * @param p1
 * @param p2
 * @note le seuil est le seuil d'erreur dans la comparaison
 * @return vrai si les normales sont similaires, faux sinon
 */
bool MainWindow::normalesAreSimilar(QVector3D p1, QVector3D p2){
    double seuil = 0.4;
    p1.normalize();
    p2.normalize();
    // cout << "-----------------------" << endl;
    // cout << p1[0] << ", " << p1[1] << ", " << p1[2] << endl;
    // cout << p2[0] << ", " << p2[1] << ", " << p2[2] << endl;

    if( (p1[0]<p2[0]+seuil && p1[0]>p2[0]-seuil) && (p1[1]<p2[1]+seuil&& p1[1]>p2[1]-seuil) && (p1[2]<p2[2]+seuil && p1[2]>p2[2]-seuil) ||
            ((-p1[0])<p2[0]+seuil && (-p1[0])>p2[0]-seuil) && ((-p1[1])<p2[1]+seuil&& (-p1[1])>p2[1]-seuil) && ((-p1[2])<p2[2]+seuil && (-p1[2])>p2[2]-seuil)){
        // cout << "sont similaires."<< endl;
        return true;
    }
    // cout << "sont pas similaires."<< endl;
    return false;
}

/**
 * @brief MainWindow::axeViaNormal - trouve l'axe d'un plan en fonction de sa normale
 * @param n - normale
 * @return 0 pour x, 1 pour y, 2 pour z
 */
int MainWindow::axeViaNormal(QVector3D n){
    n.normalize();
    double seuil = 0.6;
    // cout << "Normale : " << n[0] << ", "<< n[1] << ", "<< n[2] << endl;
    if ((n[0] > seuil || n[0] < -seuil) && (n[1] < seuil && n[1] > -seuil) && (n[2] < seuil && n[2] > -seuil)){
        // cout << "axe x" << endl;
        return 0;
    } else if ((n[0] < seuil && n[0] > -seuil) && (n[1] > seuil || n[1] < -seuil) && (n[2] < seuil && n[2] > -seuil)){
        // cout << "axe y" << endl;
        return 1;
    } else if ((n[0] < seuil && n[0] > -seuil) && (n[1] < seuil && n[1] > -seuil) && (n[2] > seuil || n[2] < -seuil)){
        // cout << "axe z" << endl;
        return 2;
    } else {
        // cout << "axe autre" << endl;
        // cout << "x : " << n[0] << ", y : " << n[1] << ", z: " << n[2] << endl;
        return 4;
    }
}

/**********************
      MODELISATION
**********************/
/**
 * @brief MainWindow::addPlane - Ajout d'un plan au viewer avec en paramètre une équation de plan
 * @param viewer
 * @param planeCoef
 * @return
 */
pcl::visualization::PCLVisualizer::Ptr  MainWindow::addPlane(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::ModelCoefficients planeCoef)
{
	this->nb_plane = this->nb_plane +1;
	viewer->addPlane(planeCoef, planeCoef.values[0],planeCoef.values[1], planeCoef.values[2], "plane"+this->nb_plane,0);
	//viewer->addPlane(planeCoef);
    return (viewer);
}

/**
 * @brief MainWindow::calc_bounding_box - calcul la boite englobante d'un nuage de point
 * @param cloud - nuage
 */
void MainWindow::calc_bounding_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    for(int i = 0; i<cloud->points.size();i++)
    {
        if(cloud->points[i].x > xmax)
            xmax = cloud->points[i].x;
        if(cloud->points[i].y > ymax)
            ymax = cloud->points[i].y;
        if(cloud->points[i].z > zmax)
            zmax = cloud->points[i].z;

        if(cloud->points[i].x < xmin)
            xmin= cloud->points[i].x;
        if(cloud->points[i].y < ymin)
            ymin = cloud->points[i].y;
        if(cloud->points[i].z < zmin)
            zmin = cloud->points[i].z;
    }
    std::cout<<"BOUNDING BOX : "<<endl<<"MAX : "
    <<" x : "<<xmax<<" y : "<<ymax<<" z : "<<zmax
    <<endl<<"MIN : "
    <<" x : "<<xmin<<" y : "<<ymin<<" z : "<<zmin<<endl;
}

/**
 * @brief MainWindow::calc_inter_planes - calcul les intersections de tous les plans
 * @note utilise room qui est la liste des plans de la piece
 */
void MainWindow::calc_inter_planes()
{
    cout << "calc" << endl;
    eq_planes.clear();
    for(int i = 0; i<room.size();i++)
    {
        double * plane_eq = equation_plane2(room.at(i));
        eq_planes.push_back(plane_eq);
    }

    cout<<"INTERSECTIONS DES PLANS : "<<endl;
    //on parcourt les plans 3 à 3
    for(int i=0;i<eq_planes.size(); i++)
    {
        for(int j=0;j<eq_planes.size();j++)
        {
            for(int k = 0;k<eq_planes.size();k++)
            {
                if(i != j && j!=k && i!=k)
                {
                    QVector3D point = resol_3eq_3inc(eq_planes.at(i), eq_planes.at(j), eq_planes.at(k));
                    if(!(point[0]>xmax+100 || point[1]>ymax+100 || point[2]>zmax+100 || point[0]<xmin-100 || point[1]<ymin-100 || point[2]<zmin-100))
                    {
                        //cout<<"x = "<<point[0]<<" y = "<<point[1]<<" z = "<<point[2]<<endl;
                        inter_points.push_back(point);
                    }
                }
            }
        }
    }
    cout<<"FIN INTERSECTIONS DES PLANS "<<endl;
}

/**********************
        DEPTHMAP
**********************/
/**
 * @brief MainWindow::plane_to_pict - création de map de profondeur (depthMap) afin de pouvoir par la suite détecter les plans contenus dans les plans trouvés
 *
 */
void MainWindow::plane_to_pict() //try to optimise
{
    //std::vector<pcl::PointCloud<pcl::PointXYZRGB>> room
    double distmax = 0.0;
    double distmoy = 0.0;
    double distmin = 999999999.9;
    for(int i = 0; i<room.size();i++)
    {

        double xmin = 999999999.9;
        double ymin = 999999999.9;
        double xmax = -999999999.9;
        double ymax = -999999999.9;
        double zmin = 999999999.9;
        double zmax = -999999999.9;
        bool scale_find = false;
        /*for(int j=0;j<room.at(i).points.size();j++)
        {
            for(int k=0;k<room.at(i).points.size();k++)
            {
                float dist_temp = sqrt(pow(room.at(i).points[k].x-room.at(i).points[j].x,2) +
                pow(room.at(i).points[k].y-room.at(i).points[j].y,2) +
                pow(room.at(i).points[k].z-room.at(i).points[j].z,2));
                if(j != k && distmax<dist_temp)
                {
                        distmax = dist_temp;

                }
                else if(j != k && distmin>dist_temp)
                {
                    distmin = dist_temp;
                }
            }
        }*/
         //échelle image / nuage de points
        pcl::PointCloud<pcl::PointXYZ> ccpy;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ccpy_ptr (new pcl::PointCloud<pcl::PointXYZ>);

        copyPointCloud (room.at(i), ccpy);
        copyPointCloud (room.at(i), *ccpy_ptr);
        double * eq = equation_plane2(ccpy);
        //orientation par rapport à l'axe y
        /*float normU1 = sqrt(pow(0,2)+pow(eq[1],2));//+pow(eq[2],2));
        float normV1 = sqrt(pow(0,2)+pow(1,2));//+pow(1,2));
        float uv1 = 0*0+eq[1]*1;
        float angle1 = acos(uv1/(normU1*normV1));
        //orientation par rapport à l'axe x
        float normU2 = sqrt(pow(0,2)+pow(eq[2],2));//+pow(eq[2],2));
        float normV2 = sqrt(pow(0,2)+pow(1,2));
        float uv2 = 0*0+eq[2]*1;
        float angle2 = acos(uv2/(normU2*normV2));
        cout<<"picture ANGLE 1 : "<<angle1<<endl;
        cout<<"picture ANGLE 2 : "<<angle2<<endl;*/
        if((1.2>eq[0] && eq[0]>0.8) ||(-1.2<eq[0] && eq[0]<-0.8))
        {
            ccpy_ptr = rotateCloud(ccpy_ptr, 90.0, 1);
            rotate_room.push_back(1);
        }
        else if((1.2>eq[1] && eq[1]>0.8) ||(-1.2<eq[1] && eq[1]<-0.8))
        {
            ccpy_ptr = rotateCloud(ccpy_ptr, 90.0, 0);
            rotate_room.push_back(0);
        }
        else
        {
            rotate_room.push_back(2);
        }

        /*else if((1.2>eq[2] && eq[2]>0.8) ||(-1.2<eq[2] && eq[2]<-0.8))
        {
            ccpy_ptr = rotateCloud(ccpy_ptr, 90.0, 0);
        }*/
        /*ccpy_ptr = rotateCloud(ccpy_ptr, angle1, 0);
        ccpy_ptr = rotateCloud(ccpy_ptr, angle2, 2);*/

        copyPointCloud (*ccpy_ptr, ccpy);
        eq = equation_plane2(ccpy);
        int div = 0;
        for(int j=0;j<ccpy.points.size();j++)
        {
            if(ccpy.points[j].x<xmin)
            {
                xmin = ccpy.points[j].x;
            }
            else if(ccpy.points[j].x>xmax)
            {
                xmax = ccpy.points[j].x;
            }
            else if(ccpy.points[j].y<ymin)
            {
                ymin = ccpy.points[j].y;
            }
            else if(ccpy.points[j].y>ymax)
            {
                ymax = ccpy.points[j].y;
            }
            else if(eq[0]*ccpy_ptr->points[j].x + eq[1]*ccpy_ptr->points[j].y+eq[2]*ccpy_ptr->points[j].z + eq[3] > zmax)
            {
                zmax = eq[0]*ccpy_ptr->points[j].x + eq[1]*ccpy_ptr->points[j].y+eq[2]*ccpy_ptr->points[j].z + eq[3];
            }
            else if(eq[0]*ccpy_ptr->points[j].x + eq[1]*ccpy_ptr->points[j].y+eq[2]*ccpy_ptr->points[j].z + eq[3] < zmin)
            {
                zmin = eq[0]*ccpy_ptr->points[j].x + eq[1]*ccpy_ptr->points[j].y+eq[2]*ccpy_ptr->points[j].z + eq[3];
            }
            if(!scale_find)
            {
                for(int k=0;k<ccpy.points.size();k++)
                {
                    float dist_temp = sqrt(pow(ccpy.points[k].x-ccpy.points[j].x,2) +
                    pow(ccpy.points[k].y-ccpy.points[j].y,2) +
                    pow(ccpy.points[k].z-ccpy.points[j].z,2));
                    if(j != k && distmax<dist_temp)
                    {
                            distmax = dist_temp;
                    }
                    else if(j != k && distmin>dist_temp)
                    {
                        distmin = dist_temp;
                    }
                }
                scale_find = true;
            }
            if(distmin>distmoy)
            {
                distmoy = distmin;
            }
            //distmoy +=distmin;
            //div++;
        }
        //float scale = 1/distmin;
        //distmoy = distmoy/div;
        //calculer la distance moyenne des 2 voisins
        //utiliser un certain nombre de minima ?
        float scale = 1/distmoy;
        int dimX = sqrt(pow(xmax - xmin,2))*scale;
        int dimY = sqrt(pow(ymax - ymin,2))*scale;
        int difx = sqrt(pow(xmin,2));
        int dify = sqrt(pow(ymin,2));
        if(zmin < 0)
        {
            zmin = -zmin;
        }
        if(zmax<0)
        {
            zmax = -zmax;
        }
        float z_scale = 0.0;
        if(zmax > zmin)
        {
            z_scale = zmax;
        }
        else
        {
            z_scale = zmin;
        }
        this->scale_depth = z_scale;

        QImage im(dimX,dimY,QImage::Format_RGB32);
        for(int x = 0; x<dimX;x++)
        {
            for(int y = 0; y<dimY;y++)
            {
                if(x> dimX-1 || y>dimY-1)
                {
                    break;
                }
                else
                {
                    QRgb value;

                    value = qRgb(255.0, 255.0, 255.0);
                    im.setPixelColor(x,y,value);
                }
            }
        }
        float * xy = new float[2];
        xy[0] = dimX;
        xy[1] = dimY;
        scale_xy.push_back(xy);
        float * dxy = new float[2];
        dxy[0] = difx;
        dxy[1] = dify;
        dif_xy.push_back(dxy);
        for(int j=0;j<ccpy_ptr->points.size();j++)
        {
            int x = ccpy_ptr->points[j].x * scale + difx*scale;
            int y = ccpy_ptr->points[j].y * scale + dify*scale;
            if(x <= dimX-1 || y <= dimY-1)
            {
                float c = eq[0]*ccpy_ptr->points[j].x + eq[1]*ccpy_ptr->points[j].y+eq[2]*ccpy_ptr->points[j].z + eq[3];
                c = c*255/z_scale;
                if(c < 0)
                {
                    c = -c;
                }
                if(c > 255.0)
                {
                    c = 255.0;
                }
                /*else if(c<(255/3))
                {
                    c = 255.0;
                }*/

                //QColor color((255.0-c),(255.0-c),(255.0-c));
                QRgb value;

                value = qRgb(c, c, c);
                im.setPixelColor(x,y,value);
                /*if(x > 0 && y > 0 && x < dimX && y < dimY)
                {
                    im.setPixelColor(x-1,y-1,value);
                    im.setPixelColor(x+1,y+1,value);
                    im.setPixelColor(x-1,y,value);
                    im.setPixelColor(x,y-1,value);
                    im.setPixelColor(x+1,y,value);
                    im.setPixelColor(x,y+1,value);
                    im.setPixelColor(x+1,y-1,value);
                    im.setPixelColor(x-1,y+1,value);
                }*/
                //eq[0]*inter_points.at(j)[0] + eq[1]*inter_points.at(j)[1]+ eq[2]*inter_points.at(j)[2] + eq[3];
            }

        }
        std::stringstream st;
        st<< i;

        std::string s = st.str();
        QString qs = QString::fromStdString(s);
        QString src = "pointcloud" + qs +".jpg";

        im.save(src,"JPEG");
    }
}

/*********************************
*
*			SLOTS
*
*********************************/

/**
 * @brief MainWindow::changeThreshold - changement de la valeur du slider threshold
 * @param th
 */
void MainWindow::changeThreshold(int th){
	this->threshold = th * 1.0f;
}

/**
 * @brief MainWindow::changeProba - changement de la valeur du slider proba
 * @param proba
 */
void MainWindow::changeProba(int proba){
	this->proba = proba / 100.0f;
}


/**
 * @brief launch_viewer - fonction permettant de lancer le viewer (créée à la base pour de la paralélisation)
 * @param v
 */
void launch_viewer(pcl::visualization::PCLVisualizer::Ptr v)
{
	while (!v->wasStopped ())
	{
		v->spinOnce (100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

/**
 * @brief MainWindow::on_action_propos_triggered - affichage de la fenetre 'A Propos' de l'application
 */
void MainWindow::on_action_propos_triggered()
{
    QMessageBox msgBox;
    msgBox.setText("Developped by: \n Colas CLAUDET, Yoann FOULON, Rachel GLAIDE");
    msgBox.exec();
}

//----------boutons de processus
/**
 * @brief MainWindow::chooseFile - sélection du fichier ply pour le traitement
 */
void MainWindow::chooseFile(){
    // initialize PointClouds
    cloud_basic = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    final = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	this->file = QFileDialog::getOpenFileName(this,
        tr("Ouvrir fichier"), "",
        tr("Polygon File Format(*.ply);;"
        "Tous les fichiers(*)"));

    if (this->file == ""){
        std::cout << "Aucun fichier choisi." << std::endl;
        return;
    }
    if (file_is_ply) { //not use
        if (pcl::io::loadPLYFile (this->file.toStdString(), *cloud_basic) < 0)  {
            qDebug() << "Error loading point cloud " << this->file;
            return;
        }
    } else {
        if (pcl::io::loadPLYFile (this->file.toStdString(), *cloud_basic) < 0)  {
            qDebug() << "Error loading point cloud " << this->file;
            return;
        }
    }
    ui->btn_segmente->setEnabled(true);
    ui->btn_displayBasic->setEnabled(true);

}
/**
 * @brief MainWindow::displayBasic - affiche un nuage de point basique
 */
void MainWindow::displayBasic()
{
    viewer = addVisualiser(); //initialisation du visualiseur
    //affichage du nuage initial
    this->viewer = simpleVis(cloud_basic);
    while (!this->viewer->wasStopped ())
    {
        this->viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    viewer->close();
}

void MainWindow::segmente()
{
    /* initialize random seed: */
    srand (time(NULL));

    qDebug() << "draw";
    std::cout << "proba " << this->proba << std::endl;
    std::cout << "threshold " << this->threshold << std::endl;
    //cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = rotateCloud(cloud_basic, 270, 0);

    cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*cloud,*cloud_xyzrgb);

    viewer = addVisualiser(); //initialisation du visualiseur

    calc_bounding_box(cloud);
    ransac_segmentation();
    while (!this->viewer->wasStopped ())
    {
        this->viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    viewer->close();
    ui->btn_draw->setEnabled(true);

}

/**
 * @brief MainWindow::draw - fonction globale pour la recherche des éléments de la piece et leur affichage
 */
void MainWindow::draw()
{
    viewer = addVisualiser(); //initialisation du visualiseur

    //calc_inter_planes();
    for(int i = 0; i < vector_cloud.size(); i++)
    {
        double * plane_eq = equation_plane2(vector_cloud.at(i));
        eq_planes.push_back(plane_eq);
    }
    //Copie du nuage pour l'appel de RepereRoom
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list;
    cout << "vector_cloud.size()" << vector_cloud.size() << endl;
    for (unsigned i = 0; i < vector_cloud.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud( vector_cloud[i], *cloud); // copyPointCloud( vector_cloud[i], *cloud_rgb); de base
        list.push_back(cloud);
    }

    repereRoom(viewer, list);

    calc_inter_planes();
    ui->btn_modelize->setEnabled(true);



    //viewer->runOnVisualisationThread();
	while (!this->viewer->wasStopped ())
	{
			this->viewer->spinOnce (100);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

/**
 * @brief MainWindow::modelize - modelise dans une GLArea une version simplifiée de la piece
 */
void MainWindow::modelize()
{
    plane_to_pict(); //TODO
    cout<<"Modelize"<<endl;
    //ui->glarea->draw_bounding_box(xmax/100.0,ymax/100.0,zmax/100.0f,xmin/100.0f,ymin/100.0f,zmin/100.0f);

    QList<Plane> pl;
    QList<Vertex> vertices;
    {
        QVector3D p1(0.0,-1.0,-1.0);
        QVector3D p2(0.0,1.0,-1.0);
        QVector3D p3(0.0,1.0,1.0);
        QVector3D p4(0.0,-1.0,1.0);
        Plane p(p1,p2,p3,p4);

        pl.push_back(p);
    }

    {
        QVector3D p1(1.0,-1.0,-1.0);
        QVector3D p2(1.0,1.0,-1.0);
        QVector3D p3(1.0,1.0,1.0);
        QVector3D p4(1.0,-1.0,1.0);
        Plane p(p1,p2,p3,p4);

        pl.push_back(p);
    }
    {
        QVector3D p1(0.0,-1.0,-1.0);
        QVector3D p2(0.0,-1.0,1.0);
        QVector3D p3(1.0,-1.0,1.0);
        QVector3D p4(1.0,-1.0,-1.0);
        Plane p(p1,p2,p3,p4);

        pl.push_back(p);
    }
    {
        QVector3D p1(0.0,-1.0,1.0);
        QVector3D p2(0.0,1.0,1.0);
        QVector3D p3(1.0,1.0,1.0);
        QVector3D p4(1.0,-1.0,1.0);
        Plane p(p1,p2,p3,p4);

        pl.push_back(p);
    }
    {
        cout<<"TRY TO MODELISE PLANE IN OPENGL UI : "<<endl;
        if(inter_points.size()>=4)
        {

            for(int i = 0; i<eq_planes.size(); i++)
            {
                std::vector<QVector3D> plane_points;

                //r = 1.0; g = 0.0; b = 0.0; a = 1.0;
                for(int j = 0; j<inter_points.size();j++)
                {
                    if((eq_planes.at(i)[0]*inter_points.at(j)[0] + eq_planes.at(i)[1]*inter_points.at(j)[1]
                        + eq_planes.at(i)[2]*inter_points.at(j)[2] + eq_planes.at(i)[3] <= 1) &&
                        (eq_planes.at(i)[0]*inter_points.at(j)[0] + eq_planes.at(i)[1]*inter_points.at(j)[1]
                        + eq_planes.at(i)[2]*inter_points.at(j)[2] + eq_planes.at(i)[3] >= -1))
                    {
                        plane_points.push_back(inter_points.at(j));
                        cout<<" POINT(S) FOUND IN PLANE n°"<<i<<endl;
                        cout<<"ax + by + cz + d = "<< eq_planes.at(i)[0]*inter_points.at(j)[0] + eq_planes.at(i)[1]*inter_points.at(j)[1]
                        + eq_planes.at(i)[2]*inter_points.at(j)[2] + eq_planes.at(i)[3]<<endl;

                    }
                }
                if(plane_points.size()>=4)
                {
                    //dans un premier temps il faut qu'on trouve la distance la plus grande entre 2 POINTS
                    int flag1 = 0;
                    int flag2 = 0;
                    float dist = 0.0;
                    for(int k = 0; k<plane_points.size();k++)
                    {
                        for(int l = 0; l<plane_points.size();l++)
                        {
                            float dist_temp = sqrt(pow(plane_points.at(k)[0]-plane_points.at(l)[0],2) +
                            pow(plane_points.at(k)[1]-plane_points.at(l)[1],2) +
                            pow(plane_points.at(k)[2]-plane_points.at(l)[2],2));
                            if(l != k && dist<dist_temp)
                            {
                                dist = dist_temp;
                                flag1 = k;
                                flag2 = l;
                            }
                        }
                    }
                    QVector3D center((plane_points.at(flag1)[0]+plane_points.at(flag2)[0])/2,
                    (plane_points.at(flag1)[1]+plane_points.at(flag2)[1])/2,
                    (plane_points.at(flag1)[2]+plane_points.at(flag2)[2])/2);
                    dist = 0.0;
                    int flag3 = 0;
                    //for(int l = 0; l<plane_points.size();l++)
                    //{
                        //float dist_temp = sqrt(pow(center[0]-plane_points.at(l)[0],2) +
                        //pow(center[1]-plane_points.at(l)[1],2) +
                        //pow(center[2]-plane_points.at(l)[2],2));
                        //if(l != flag1 && dist<dist_temp && l != flag2)
                        //{
                            //dist = dist_temp;
                            //flag3 = l;
                        //}
                    //}
                    for(int l = 0; l<plane_points.size();l++)
                    {
                        float dist_temp = sqrt(pow(plane_points.at(flag1)[0]-plane_points.at(l)[0],2) +
                        pow(plane_points.at(flag1)[1]-plane_points.at(l)[1],2) +
                        pow(plane_points.at(flag1)[2]-plane_points.at(l)[2],2)) +
                        sqrt(pow(plane_points.at(flag2)[0]-plane_points.at(l)[0],2) +
                        pow(plane_points.at(flag2)[1]-plane_points.at(l)[1],2) +
                        pow(plane_points.at(flag2)[2]-plane_points.at(l)[2],2));
                        if(l != flag1 && dist<dist_temp && l != flag2)
                        {
                            dist = dist_temp;
                            flag3 = l;
                        }
                    }
                    dist = 0.0;
                    int flag4 = 0;
                    for(int l = 0; l<plane_points.size();l++)
                    {
                        float dist_temp = sqrt(pow(plane_points.at(flag3)[0]-plane_points.at(l)[0],2) +
                        pow(plane_points.at(flag3)[1]-plane_points.at(l)[1],2) +
                        pow(plane_points.at(flag3)[2]-plane_points.at(l)[2],2));
                        if(l != flag1 && dist<dist_temp && l != flag2 && l != flag3)
                        {
                            dist = dist_temp;
                            flag4 = l;
                        }
                    }
                    if(flag1 == flag2 || flag1 == flag3 || flag1 == flag4 || flag2 == flag3 || flag2 == flag4 || flag3 == flag4)
                    {
                        cout<<"_______PROBLEM OF FLAG_______"<<endl;
                    }
                    cout<<eq_planes.at(i)[0]<<"x + "<<eq_planes.at(i)[1]<<"y + "<<eq_planes.at(i)[2]<<"z + "<<eq_planes.at(i)[3]<<endl;
                    cout<<" FLAG 1 : "<<flag1<<" FLAG 2 : "<<flag2<<" FLAG 3 : "<<flag3<<" FLAG 4 : "<<flag4<<endl;
                    Plane pt(plane_points.at(0)/100.0,plane_points.at(2)/100.0,plane_points.at(1)/100.0,plane_points.at(3)/100.0);
                    Plane p(plane_points.at(flag1)/100.0,plane_points.at(flag3)/100.0,plane_points.at(flag2)/100.0,plane_points.at(flag4)/100.0);
                    pl.push_back(p); //pour afficher les plans
                    //pl.push_back(pt);
                }
                else
                {
                    cout<<"ERROR plane points : n°"<<plane_points.size()<<endl;
                }
                plane_points.clear();
            }
        }
        else
        {
             cout<<"ERROR : NO INTER POINTS"<<endl;
        }
    }
    for(int i = 0; i<inter_points.size();i++)
    {
        Vertex v(0.30,inter_points.at(i)[0]/100.0,inter_points.at(i)[1]/100.0,inter_points.at(i)[2]/100.0);
        //v.setColor((rand()%255)/255.0,(rand()%255)/255.0,(rand()%255)/255.0,(rand()%255)/255.0);
        vertices.push_back(v);//to decoche
    }
    for(int i = 0; i < cloud_xyzrgb->points.size();i = i + 10 )
    {
        Vertex v(0.05,cloud_xyzrgb->points[i].x/100.0,cloud_xyzrgb->points[i].y/100.0,cloud_xyzrgb->points[i].z/100.0);
        v.setColor(1.0,1.0,1.0,1.0);
        vertices.push_back(v);//to decoche
    }
    ui->glarea->addVertex(vertices);
    ui->glarea->addPlanes(pl);
    ui->glarea->draw_bounding_box(xmax/100.0,ymax/100.0,zmax/100.0f,xmin/100.0f,ymin/100.0f,zmin/100.0f);

    //ui->glarea->draw_bounding_box(1.0f,1.0f,1.0f,-1.0f,-1.0f,-1.0f);
}
