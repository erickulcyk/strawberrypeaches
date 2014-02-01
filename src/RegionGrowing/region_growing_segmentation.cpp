#include "stdafx.h"
#include <vector>
using namespace std;

//convenient typedefs
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//convenient structure to handle our pointclouds
struct PCD
{
	PointCloud::Ptr cloud;
	std::string f_name;

	PCD() : cloud (new PointCloud) {};
};

struct callback_args{
	// structure used to pass arguments to the callback function
	PointCloudT::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
 * \param argc the number of arguments (pass from main ())
 * \param argv the actual command line arguments (pass from main ())
 * \param models the resultant vector of point cloud datasets
 */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
	std::string extension (".pcd");
	// Suppose the first argument is the actual test model
	for (int i = 3; i < argc; i++)
	{
		std::string fname = std::string (argv[i]);
		// Needs to be at least 5: .plot
		if (fname.size () <= extension.size ())
			continue;

		std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

		//check that the argument is a pcd file
		if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
		{
			// Load the cloud and saves it into the global list of models
			PCD m;
			m.f_name = argv[i];
			pcl::io::loadPCDFile (argv[i], *m.cloud);
			double avx=0, avy=0,avz=0;
			int size = m.cloud->points.size();
			for(int j=0 ; j < size; j++)
			{
				PointT point = m.cloud->points[j];
				avx+=point.x;
				avy+=point.y;
				avz+=point.z;
			}

			avx/=size;
			avy/=size;
			avz/=size;	
			for(int j=0 ; j < size; j++)
			{
				m.cloud->points[j].x-=avx;
				m.cloud->points[j].y-=avy;
				m.cloud->points[j].z-=avz;
			}

			//remove NAN points from the cloud
			std::vector<int> indices;
			// pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

			models.push_back (m);
		}
	}

	cout<<"Read in: "<<(argc-3)<<" clouds."<<endl;
}

void ComputeNormals
(
	pcl::PointCloud<PointT>::Ptr cloud,
	pcl::PointCloud<pcl::Normal>& normals
)
{
	pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
	pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (cloud);
	//normal_estimator.setKSearch (50);
	normal_estimator.setRadiusSearch (0.1);
	normal_estimator.compute (normals);
}

void ExtractRegions
(
	pcl::PointCloud<PointT>::Ptr cloud,
	pcl::PointCloud<pcl::Normal>::Ptr normals,
	std::vector <pcl::PointIndices>& clusters,
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr& coloredCloud,
	double smoothThreshold,
	double curveThreshold
)
{
	pcl::RegionGrowing<PointT, pcl::Normal> reg;
	reg.setMinClusterSize (50);
	//reg.setMaxClusterSize (10000);
	pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (30);
	reg.setInputCloud (cloud);
	//reg.setIndices (indices);
	reg.setInputNormals (normals);
	reg.setSmoothnessThreshold (smoothThreshold);
	reg.setCurvatureThreshold (curveThreshold);

	reg.extract (clusters);
	coloredCloud = reg.getColoredCloud();
}

void MouseCallback(const pcl::visualization::MouseEvent &mouseEvent, void *cookie)
{

}

void pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
	struct callback_args* data = (struct callback_args *)args;
	cout<<"In point callback method"<<endl;
	if (event.getPointIndex () == -1)
		return;
	cout<<"Something selected!"<<endl;

	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	data->clicked_points_3d->points.push_back(current_point);
	// Draw clicked points in red:
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 255, 255);
	data->viewerPtr->removePointCloud("clicked_points");
	data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
	cout<<"Done!"<<endl;
}

int main (int argc, char** argv)
{
	double smoothThreshold = 7.0/180.0*M_PI;
	double curveThreshold = 1.0;

	if(argc>1)
	{
		smoothThreshold = atof(argv[1]);
	}

	if(argc>2)
	{
		curveThreshold = atof(argv[2]);
	}

	vector<PCD, Eigen::aligned_allocator<PCD> > models;
	loadData(argc, argv, models);
	pcl::PointCloud<PointT>::Ptr cloud = models[0].cloud;
	
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	ComputeNormals(cloud, *normals);
	cout<<"Computed Normals"<<endl;

	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.0);
	pass.filter (*indices);
	cout<<"Done with filter!"<<endl;
	
	std::vector <pcl::PointIndices> clusters;
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr coloredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	ExtractRegions(cloud, normals, clusters, coloredCloud, smoothThreshold, curveThreshold);
	cout<<"Extracted Clusters!"<<endl;

	std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(coloredCloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (coloredCloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	//viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (colored_cloud, normals, 10, 0.05, "normals");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();	

	pcl::PolygonMesh deskMesh;
	pcl::io::loadPolygonFileSTL ("../data/desk.stl",deskMesh);
	viewer->addPolygonMesh(deskMesh);

	pcl::PolygonMesh polyMesh;
	pcl::io::loadPolygonFileSTL ("../data/sample.stl",polyMesh);
	//viewer->addPolygonMesh(polyMesh);
	pcl::PointCloud<PointT> cloud2;
	pcl::fromPCLPointCloud2(polyMesh.cloud, cloud2);
	
	float scale = .1;
	Eigen::Matrix4f a;
	a<<
		scale,	.0,	.0,	.0,
		.0,	scale,	.0,	.0,
		.0, 	.0,	scale,	.0,
		.0, 	.0,  	.0,	1.0;

	pcl::transformPointCloud(cloud2,cloud2, a);
	pcl::toPCLPointCloud2(cloud2, polyMesh.cloud);
	//viewer->updatePolygonMesh(polyMesh);
	viewer->registerMouseCallback(MouseCallback);
	struct callback_args cb_args;
	PointCloudT::Ptr clicked_points_3d (new PointCloudT);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewerPtr = viewer;
	viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return (0);
}
