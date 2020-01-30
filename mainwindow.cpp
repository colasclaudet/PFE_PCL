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
    bool file_is_ply = false;
	
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
	//-- end load cloud
    if(this->view_plan)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        copyPointCloud(*cloud,*cloud_xyzrgb); //initialisation des points avec les couleur se passe correctement ? apparement non donc il va falloir set les couleurs des points à 0

        /*for (pcl::PointCloudT::iterator cloud_it = cloud_xyzrgb->begin (); cloud_it != cloud_xyzrgb->end (); ++cloud_it)
        {
            cloud_it->r = 0;
            cloud_it->g = 0;
            cloud_it->b = 0;
        }*/
        for (size_t i=0;i<cloud_xyzrgb->points.size();i++)
        {
            cloud_xyzrgb->points[i].r = 255;
            cloud_xyzrgb->points[i].g = 50;
            cloud_xyzrgb->points[i].b = 100;
        }

        /*cout<<"nb_points : "<<cloud_xyzrgb->points.size()<<endl;
        for(int i = 0; i<10; i++)
        {
            float a = (rand()%10000)/100.0f;
            float b = (rand()%100)/100.0f;
            float c = (rand()%200)/100.0f;
            float d = (rand()%200)/100.0f;
            if(c>=d)
            {
                float temp = d;
                d=c;
                c=temp;
            }
            don_segmentation2(cloud_xyzrgb, a, 0.25,0.02, 2.0); //always in the range (0,1) 0.25, 0.25,0.2, 0.5
            cout<<" a "<<a<<" b "<<b<<" c "<<c<<" d "<<d<<endl;
        }*/
        don_segmentation(cloud_xyzrgb, 40.25, 40.25,40.2, 40.5); //always in the range (0,1) 0.25, 0.25,0.2, 0.5
        cloud->clear();

        this->viewer = simpleVis(cloud);
        this->nb_cloud++;
        for(int i = 0; i<vector_cloud_RGB.size();i++)
        {
            //this->nb_cloud++;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
            copyPointCloud(this->vector_cloud_RGB.at(i),*tmp);
            cout<<"cloud n°"<<i<<" size :"<<tmp->points.size();
            this->viewer = addPtsCloudXYZRGB(viewer,tmp);
        }
        //this->viewer = addPtsCloudXYZRGB (viewer, cloud_xyzrgb);
        //cout << "nb plan :" << this->nb_cloud << endl;

        //finaliseVis(viewer);
        while (!this->viewer->wasStopped ())
        {
            this->viewer->spinOnce (100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    else
    {
        this->viewer = simpleVis(cloud);
        this->nb_cloud++;

        //cout << "nb plan :" << this->nb_cloud << endl;

        //finaliseVis(viewer);
        while (!this->viewer->wasStopped ())
        {
            this->viewer->spinOnce (100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void MainWindow::modelize()
{
    cout<<"Modelize"<<endl;

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

    ui->glarea->addPlanes(pl);
}

void MainWindow::on_action_propos_triggered()
{
    QMessageBox msgBox;
    msgBox.setText("Developped by: \n Colas CLAUDET, Yoann FOULON, Rachel GLAIDE");
    msgBox.exec();
}
