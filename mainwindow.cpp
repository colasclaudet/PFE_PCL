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
	connect(ui->radio_cloud, SIGNAL(clicked()), this, SLOT(chooseViewCloud())); //si cloud radio button sélectionné
	connect(ui->radio_plane, SIGNAL(clicked()), this, SLOT(chooseViewPlane())); //si plane radio button selectionné
	connect(ui->slider_threshold, SIGNAL(valueChanged(int)), this, SLOT(changeThreshold(int))); //connexion slidebar threshold
	connect(ui->slider_proba, SIGNAL(valueChanged(int)), this, SLOT(changeProba(int))); //connexion slidebar proba
	connect(ui->btn_import, SIGNAL(clicked()), this, SLOT(chooseFile())); //connexion selection de fichier
	connect(ui->btn_draw, SIGNAL(clicked()), this, SLOT(draw())); //connexion bouton draw, on lance le viewer
    connect(ui->btn_modelize, SIGNAL(clicked()), this, SLOT(modelize())); //connexion bouton modelize, on lance la modelisation

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
    cout<<"ADD POINTCLOUD N°"<<this->nb_cloud<<endl;
	viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud"+this->nb_cloud);
	this->nb_cloud = this->nb_cloud +1;
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

pcl::visualization::PCLVisualizer::Ptr MainWindow::addPtsCloudXYZRGB (pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // -----------------------------------------------
    // -----Open 3D viewer and add color point cloud--
    // -----------------------------------------------
    float r = rand()%255;
    float g = rand()%255;
    float b = rand()%255;
    for (size_t i=0;i<cloud->points.size();i++)
    {
        cloud->points[i].r = r;
        cloud->points[i].g = g;
        cloud->points[i].b = b;
    }
    cout<<"-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-"<<endl;
    cout<<"ADD COLOR POINTCLOUD N°"<<this->nb_cloud<<" SIZE : "<<cloud->points.size()<<" R : "<<r<<" G : "<<g<<" B : "<<b<<endl;
    cout<<"-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-"<<endl;
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud"+this->nb_cloud);
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

//debruitage
void MainWindow::denoise(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
    std::cerr << "denoising running " << std::endl;

    //creer nuage inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
    // creer nuage outliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // creer cloud pour futur nuage filtré
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
/**********************
      PFE
**********************/
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
    for(int p = 0; p<3;p++)
        for(int l = 0; l<4; l++)
            //qDebug() <<"matrice : "<<matrice[p][l]<<endl;
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
    //qDebug()  << "X est egal a " << x << "\n";
    //qDebug()  << "Y est egal a " << y << "\n";
    //qDebug()  << "Z est egal a " << z << "\n";
    QVector3D point(x,y,z);
    //system("PAUSE");
    return point;

}
void MainWindow::don_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, double angle, double threshold, double scale1, double scale2)
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

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;

    if (cloud->isOrganized ())
    {
        tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB> ());
    }
    else
    {
      tree.reset (new pcl::search::KdTree<pcl::PointXYZRGB> (false));
    }

    tree->setInputCloud (cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr small_cloud_downsampled;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr large_cloud_downsampled;

    // If we are using approximation
    if(approx)
    {
        std::cout << "Downsampling point cloud for approximation" << std::endl;

        // Create the downsampling filtering object
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setDownsampleAllData (false);
        sor.setInputCloud (cloud);

        // Create downsampled point cloud for DoN NN search with small scale
        small_cloud_downsampled = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        float smalldownsample = static_cast<float> (scale1 / decimation);
        sor.setLeafSize (smalldownsample, smalldownsample, smalldownsample);
        sor.filter (*small_cloud_downsampled);
        std::cout << "Using leaf size of " << smalldownsample << " for small scale, " << small_cloud_downsampled->size() << " points" << std::endl;

        // Create downsampled point cloud for DoN NN search with large scale
        large_cloud_downsampled = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        const float largedownsample = float (scale2/decimation);
        sor.setLeafSize (largedownsample, largedownsample, largedownsample);
        sor.filter (*large_cloud_downsampled);
        std::cout << "Using leaf size of " << largedownsample << " for large scale, " << large_cloud_downsampled->size() << " points" << std::endl;

        this->viewer = addPtsCloudXYZRGB (viewer, large_cloud_downsampled); //ADDVIEWER
    }

    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
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
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::PointNormal> don;
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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_don_color (new pcl::PointCloud<pcl::PointXYZRGB>);
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
    // initialize PointClouds
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
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
        if (pcl::io::loadPLYFile (this->file.toStdString(), *cloud) < 0)  {
            qDebug() << "Error loading point cloud " << this->file;
            return;
        }
    } else {
        if (pcl::io::loadPLYFile (this->file.toStdString(), *cloud) < 0)  {
            qDebug() << "Error loading point cloud " << this->file;
            return;
        }
    }
}

//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-
//fonction mère , appelle les autres fonctions et gère l'affichage
//-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-

void MainWindow::ransac_segmentation()
{
    //debruitage
    denoise(cloud); //on enlève le bruit du nuage de points
     //le minimum de points que doit comporter un nuage de points pour être conservé
    pcl::io::loadPLYFile ("./test_noise_inliers.ply", *cloud); //récupération du fichier dans lequel il y a le nuage de point débruité
    //mettre le nuage de point en couleur et creer un viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 0, 255); //initialisation d'une couleur bleue pour les points
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cpy = cloud;

    int maxToDelete = 0.02*cloud->size();
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

        // creates the visualization object and adds either our original cloud or all of the inliers
        // depending on the command line arguments specified.
        //std::cout << "Viewer" << endl;

        //récupération des équations des plans
        if(final->size() > maxToDelete) //si la taille du plan est suppérieure à un pourcentage de la taille du nuage de point global donné
        {
            std::vector<float> list_coeff; //liste des coefs de l'équation du plan
            std::vector<float> sum; //vector d'addition des coefs pour en faire une moyenne
            for(int i = 0; i < 4; ++i)
                sum.push_back(0.0); //init sum

            int nb_iter_for_avg = 1; //nombre de fois qu'on itere pour faire la moyenne des equations des plans

            for(int i= 0; i < nb_iter_for_avg; ++i)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d (final); //  creer nuage (copie de final)
                Eigen::VectorXf ground_coeffs;
                ground_coeffs.resize(4); // coefficient du plan
                std::vector<int> clicked_points_indices; //indices des points du plan  choisis random

                int a = 0; //indice premier point
                int b = 0; // ...second point
                int c = 0; //... troisième point
                // creer des points randoms
                // on sélectionne trois points random dans notre plan
                while(a==b || b==c || a==c)
                {
                    a = (rand() % final->size()) ;
                    b = (rand() % final->size()) ;
                    c = (rand() % final->size()) ;
                }
                std::cout<<"A "<<a<<"B " <<b<<"C "<<c<<endl;
                // les mettre dans clicked_point_indices
                clicked_points_indices.push_back(a);
                clicked_points_indices.push_back(b);
                clicked_points_indices.push_back(c);

                //pcl::SampleConsensusModelPlane<pcl::PointXYZ> model_plane(clicked_points_3d);
                pcl::SampleConsensusModelPlane<pcl::PointXYZ> model_plane(final);
                model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);// calcul plan a partir des indices des points

                //ici on va calculer les équations de plan
                QVector3D p1(final->points[a].x,final->points[a].y,final->points[a].z);
                QVector3D p2(final->points[b].x,final->points[b].y,final->points[b].z);
                QVector3D p3(final->points[c].x,final->points[c].y,final->points[c].z);
                double * plane_eq = equation_plane(p1,p2,p3);
                eq_planes.push_back(plane_eq);
                std::cout<<"PLANE EQUATION : "<<plane_eq[0]<<"x + "<<plane_eq[1]<<"y + "<<plane_eq[2]<<"z + "<<plane_eq[3]<<" = 0"<<endl;
                //end
                std::cout << "Ground plane: " << ground_coeffs(0)
                        << " " << ground_coeffs(1) << " " << ground_coeffs(2)
                        << " " << ground_coeffs(3) << std::endl;

                // faire la somme des coeffs pour chaque boucle effectuée
                sum[0] += ground_coeffs(0);
                sum[1] += ground_coeffs(1);
                sum[2] += ground_coeffs(2);
                sum[3] += ground_coeffs(3);
            }
            //calcul de la moyenne pour chaque valeur de l equation
            int iter_eq = 4;
            for(int i = 0; i < iter_eq; ++i)
                list_coeff.push_back(sum[i]);
                //list_coeff.push_back(sum[i]/nb_iter_for_avg);

            std::cout << "plane : " << list_coeff[0]
            << " " << list_coeff[1] << " " << list_coeff[2]
            << " " << list_coeff[3] << std::endl;
            //for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
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

            vector_cloud.push_back(*final); //on ajoute le nuage de plan ransac au vecteur mais inutilisé //marche pas car les finals sont les memes
            cout << "Taille vector_cloud : " << vector_cloud.size();
        }

        //ici on supprime du nuage de point original le plan trouvé par ransac
        // comme ça lors de la prochaine itération, ransac sélectionne un autre plan
        extract.setInputCloud(cloud);
        extract.setIndices(suppression_inliers);
        extract.setNegative(true);
        extract.filter(*cloud);

        have_plane = true; //il y a au moins un plan trouvé par ransac
    }
}

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

void MainWindow::draw()
{
	/* initialize random seed: */
	srand (time(NULL));

	qDebug() << "draw";
	std::cout << "proba " << this->proba << std::endl;
	std::cout << "threshold " << this->threshold << std::endl;
    cloud = rotateCloud(cloud, 270, 0);
    

    viewer = addVisualiser(); //initialisation du visualiseur

    if(this->view_plan) //a modifier sur le long terme
    {

        /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        copyPointCloud(*cloud,*cloud_xyzrgb); //initialisation des points avec les couleur se passe correctement ? apparement non donc il va falloir set les couleurs des points à 0

        for (size_t i=0;i<cloud_xyzrgb->points.size();i++)
        {
            cloud_xyzrgb->points[i].r = 255;
            cloud_xyzrgb->points[i].g = 50;
            cloud_xyzrgb->points[i].b = 100;
        }
        don_segmentation(cloud_xyzrgb, 40.25, 40.25,40.2, 40.5); //always in the range (0,1) 0.25, 0.25,0.2, 0.5
        copyPointCloud( *cloud_xyzrgb, *cloud);*/
        //cloud->clear();
        
        //this->viewer = simpleVis(cloud);

        /*this->nb_cloud++;
        for(int i = 0; i<vector_cloud_RGB.size();i++)
        {
            //this->nb_cloud++;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
            copyPointCloud(this->vector_cloud_RGB.at(i),*tmp);
            cout<<"cloud n°"<<i<<" size :"<<tmp->points.size();
            this->viewer = addPtsCloudXYZRGB(viewer,tmp);
        }*/
        //this->viewer = addPtsCloudXYZRGB (viewer, cloud_xyzrgb);
        //cout << "nb plan :" << this->nb_cloud << endl;
        //finaliseVis(viewer);

        //debut fonction
        calc_bounding_box(cloud);
        ransac_segmentation();
        //fin fonction
        cout<<"INTERSECTIONS DES PLANS : "<<endl;
        for(int i=0;i<eq_planes.size()-2;i++)
        {
            for(int j=0;j<eq_planes.size()-2;j++)
            {
                for(int k = 0;k<eq_planes.size();k++)
                {
                    if(i != j && j!=k && i!=k)
                    {
                        QVector3D point = resol_3eq_3inc(eq_planes.at(i), eq_planes.at(j), eq_planes.at(k));

                        if(!(point[0]>xmax || point[1]>ymax || point[2]>zmax || point[0]<xmin || point[1]<ymin || point[2]<zmin))
                        {
                            cout<<"x = "<<point[0]<<" y = "<<point[1]<<" z = "<<point[2]<<endl;
                            inter_points.push_back(point);
                        }

                    }
                }
            }
        }
        cout<<"FIN INTERSECTIONS DES PLANS "<<endl;

    }
    else //affichage du nuage initial
    {
        this->viewer = simpleVis(cloud);
        this->nb_cloud++;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> list;
    cout << "vector_cloud.size()" << vector_cloud.size() << endl;
    for (unsigned i = 0; i < vector_cloud.size(); i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        copyPointCloud( vector_cloud[i], *cloud_rgb); // copyPointCloud( vector_cloud[i], *cloud_rgb); de base
        cout << "cloud_rgb.size()" << cloud_rgb->size() << endl;
        for(size_t j=0; j < cloud_rgb->size(); j++){
            cloud_rgb->points[j].r = 255;
            cloud_rgb->points[j].g = 255;
            cloud_rgb->points[j].b = 255;
        }
        list.push_back(cloud_rgb);
    }
    repereRoom(viewer, list);


    while (!this->viewer->wasStopped ())
    {
        this->viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


}

void MainWindow::modelize()
{
    cout<<"Modelize"<<endl;
    //ui->glarea->draw_bounding_box(xmax/100.0,ymax/100.0,zmax/100.0f,xmin/100.0f,ymin/100.0f,zmin/100.0f);

    QList<Plane> pl;
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
            /*for(int i = 0; i<inter_points.size()-3;i++)
            {
                Plane p(inter_points.at(i)/100.0,inter_points.at(i+1)/100.0,inter_points.at(i+2)/100.0,inter_points.at(i+3)/100.0);
                pl.push_back(p);
            }*/

            for(int i = 0; i<eq_planes.size(); i++)
            {
                std::vector<QVector3D> plane_points;
                for(int j = 0; j<inter_points.size();j++)
                {
                    if(eq_planes.at(i)[0]*inter_points.at(j)[0] + eq_planes.at(i)[1]*inter_points.at(j)[1]
                            + eq_planes.at(i)[2]*inter_points.at(j)[2] + eq_planes.at(i)[3] <= 100 &&
                            eq_planes.at(i)[0]*inter_points.at(j)[0] + eq_planes.at(i)[1]*inter_points.at(j)[1]
                                                        + eq_planes.at(i)[2]*inter_points.at(j)[2] + eq_planes.at(i)[3] >= -100)
                    {
                        plane_points.push_back(inter_points.at(j));
                        cout<<" POINT(S) FOUND IN PLANE n°"<<i<<endl;
                    }
                }
                if(plane_points.size()>=4)
                {
                    Plane p(plane_points.at(0)/100.0,plane_points.at(1)/100.0,plane_points.at(2)/100.0,plane_points.at(3)/100.0);
                    pl.push_back(p);
                }
                else
                {
                    cout<<"ERROR plane points : °"<<plane_points.size()<<endl;
                }
            }
        }
        else
        {
            cout<<"ERROR : NO INTER POINTS"<<endl;
        }
    }
    QList<Vertex> vertices;
    for(int i = 0; i<inter_points.size();i++)
    {
        Vertex v(0.5,inter_points.at(i)[0]/100.0,inter_points.at(i)[1]/100.0,inter_points.at(i)[2]/100.0);
        vertices.push_back(v);
    }
    ui->glarea->addVertex(vertices);
    ui->glarea->addPlanes(pl);
    ui->glarea->draw_bounding_box(xmax/100.0,ymax/100.0,zmax/100.0f,xmin/100.0f,ymin/100.0f,zmin/100.0f);
    //ui->glarea->draw_bounding_box(1.0f,1.0f,1.0f,-1.0f,-1.0f,-1.0f);
}

void MainWindow::on_action_propos_triggered()
{
    QMessageBox msgBox;
    msgBox.setText("Developped by: \n Colas CLAUDET, Yoann FOULON, Rachel GLAIDE");
    msgBox.exec();
}

//http://pointclouds.org/documentation/tutorials/matrix_transform.php
pcl::PointCloud<pcl::PointXYZ>::Ptr MainWindow::rotateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int degrees, int axe)
{
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
    return transformed_cloud;
}

void MainWindow::repereRoom(pcl::visualization::PCLVisualizer::Ptr viewer, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> list_planes){
    cout<< "Repere room" << endl;
    std::vector<QVector3D> list_positions_mean;
    QVector3D pos_min = QVector3D(FLT_MAX, FLT_MAX, FLT_MAX);
    QVector3D pos_max = QVector3D(FLT_MIN, FLT_MIN, FLT_MIN);

    cout<< "nombre plan total" << list_planes.size() << endl;
    for (unsigned i = 0; i < list_planes.size(); i++)
    {
        int cpt = 0;
        QVector3D pos_mean_plane = QVector3D(0,0,0);
        //calcul de la position moyenne de chaque plans
        cout<< "list_planes[i]->points.size()" << list_planes[i]->points.size() << endl;
        for (size_t j=0;j< list_planes[i]->points.size();j++)
        {
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
    for (unsigned i = 0; i < list_positions_mean.size(); i++)
    {
        //sol
        cout<< "list_positions_mean[" <<i<<"][1]" << list_positions_mean[i].y()<< endl; 
        cout<< "pos min" << pos_min.y() << endl;
        cout<< "(pos_min[1]+error)" << (pos_min.y()+error) << endl;
        if (list_positions_mean[i].y() >= pos_min.y() && list_positions_mean[i].y() <= (pos_min.y()+error))
        {
            list_floor.push_back(list_planes[i]);
        }
        //plafond
        if (list_positions_mean[i].y() <= pos_max.y() && list_positions_mean[i].y() >= (pos_max.y()+error))
        {
            list_ceiling.push_back(list_planes[i]);
        }
    }

    cout<< "nombre plan floor" << list_floor.size() << endl;
    cout<< "nombre plan ceiling" << list_ceiling.size() << endl;

    //ajout au viewer des élements dans des couleurs différentes
    for (unsigned i = 0; i < list_floor.size(); i++)
    {
        for(size_t j=0; j < list_floor[i]->size(); j++)
        {
            list_floor[i]->points[j].r = 255;
            list_floor[i]->points[j].g = 0;
            list_floor[i]->points[j].b = 0;
        }
        addPtsCloudXYZRGB(viewer, list_floor[i]);
    }
    for (unsigned i = 0; i < list_ceiling.size(); i++)
    {
        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color (list_ceiling[i], 0, 0, 255);
        for(size_t j=0; j < list_ceiling[i]->size(); j++)
        {
            list_ceiling[i]->points[j].r = 0;
            list_ceiling[i]->points[j].g = 255;
            list_ceiling[i]->points[j].b = 0;
        }
        addPtsCloudXYZRGB(viewer, list_ceiling[i]);
    }
}
