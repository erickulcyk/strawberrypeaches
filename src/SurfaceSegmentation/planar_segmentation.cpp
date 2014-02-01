#include "stdafx.h"

//convenient typedefs
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
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
	for (int i = 1; i < argc; i++)
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
			//remove NAN points from the cloud
			std::vector<int> indices;
			// pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

			models.push_back (m);
		}
	}

	cout<<"Read in: "<<(argc-1)<<" clouds."<<endl;
}

	int
main (int argc, char** argv)
{
	if(argc<2)
	{
		cout<<"Usage: planar_segmentation.exe [*].pcd"<<endl;
		return 0;
	}

	vector<PCD, Eigen::aligned_allocator<PCD> > models;
	loadData(argc, argv, models);
/*
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// Fill in the cloud data
	cloud.width  = 15;
	cloud.height = 1;
	cloud.points.resize (cloud.width * cloud.height);

	// Generate the data
	for (size_t i = 0; i < cloud.points.size (); ++i)
	{
		cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud.points[i].z = 1.0;
	}

	// Set a few outliers
	cloud.points[0].z = 2.0;
	cloud.points[3].z = -2.0;
	cloud.points[6].z = 4.0;

	std::cerr << "Point cloud data: " << cloud.points.size () << " points" << std::endl;
	for (size_t i = 0; i < cloud.points.size (); ++i)
		std::cerr << "    " << cloud.points[i].x << " " 
			<< cloud.points[i].y << " " 
			<< cloud.points[i].z << std::endl;
*/
	// Create the filtering object
	pcl::ExtractIndices<PointT> extract;

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.09);
	
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr rest_of_cloud (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr complete_cloud (new pcl::PointCloud<PointT>);


	auto cloud0 = models[0].cloud;
	seg.setInputCloud (cloud0);
	seg.segment (*inliers, *coefficients);

	extract.setInputCloud(cloud0);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_filtered);

	for(int i =0; i < cloud_filtered->points.size();i++)
	{
		cloud_filtered->points[i].g = 255;
		cloud_filtered->points[i].r = 0;
		cloud_filtered->points[i].b = 0;
	}

	*complete_cloud+=*cloud_filtered;

	extract.setNegative(true);
	extract.filter(*rest_of_cloud);

	seg.setInputCloud (rest_of_cloud);
	seg.segment (*inliers, *coefficients);

	extract.setInputCloud(rest_of_cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_filtered);

	for(int i =0; i < cloud_filtered->points.size();i++)
	{
		cloud_filtered->points[i].g = 0;
		cloud_filtered->points[i].r = 255;
		cloud_filtered->points[i].b = 0;
	}

	*complete_cloud+=*cloud_filtered;

	extract.setNegative(true);
	pcl::PointCloud<PointT>::Ptr rest_of_cloud2 (new pcl::PointCloud<PointT>);
	extract.filter(*rest_of_cloud2);
	cout<<"Extra points: "<<rest_of_cloud2->points.size()<<endl;
	*complete_cloud+=*rest_of_cloud2;
/*
	if (inliers->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return (-1);
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " " 
		<< coefficients->values[3] << std::endl;

	std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

	for (size_t i = 0; i < inliers->indices.size (); ++i)
	{
		models[0].cloud->points[inliers->indices[i]].g = 255;
		models[0].cloud->points[inliers->indices[i]].r = 0;
		models[0].cloud->points[inliers->indices[i]].b = 0;

		if(models.size()>1)
		{
			models[1].cloud->points[inliers->indices[i]].g = 0;
			models[1].cloud->points[inliers->indices[i]].r = 255;
			models[1].cloud->points[inliers->indices[i]].b = 0;
		}

		if(models.size()>1)
		{
			models[1].cloud->points[inliers->indices[i]].g = 0;
			models[1].cloud->points[inliers->indices[i]].r = 255;
			models[1].cloud->points[inliers->indices[i]].b = 0;
		}
	}*/
	
	pcl::visualization::CloudViewer viewer ("HEY!");
	viewer.showCloud (complete_cloud);
	while (!viewer.wasStopped ()) {}

	/*
		std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
			<< cloud.points[inliers->indices[i]].y << " "
			<< cloud.points[inliers->indices[i]].z << std::endl;
*/
	return (0);
}
