#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>

void ShowCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, int index)
{
	stringstream title;
       	title << "Cloud "<<index;
	pcl::visualization::CloudViewer viewer (title.str());
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ())
	{
	}
}

	int
main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGBA>);

	// Fill in the CloudIn data
	cloud_in->width    = 10;
	cloud_in->height   = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize (cloud_in->width * cloud_in->height);
	for (size_t i = 0; i < cloud_in->points.size (); ++i)
	{
		cloud_in->points[i].x = 3 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = 3 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = 3 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].r = 255;
		cloud_in->points[i].g = 0;
		cloud_in->points[i].b = 0;
	}
	std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
		<< std::endl;
	ShowCloud(cloud_in,0);
//	for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
//		cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
//			cloud_in->points[i].z << std::endl;
	*cloud_out = *cloud_in;
	std::cout << "size:" << cloud_out->points.size() << std::endl;
	for (size_t i = 0; i < cloud_in->points.size (); ++i)
	{
		cloud_out->points[i].x = cloud_in->points[i].x + 10.0f;
		cloud_out->points[i].g = 255;
		cloud_out->points[i].r = 0;
	}
		std::cout << "Transformed " << cloud_in->points.size () << " data points:"
		<< std::endl;
	ShowCloud(cloud_out,0);
//	for (size_t i = 0; i < cloud_out->points.size (); ++i)
//		std::cout << "    " << cloud_out->points[i].x << " " <<
//			cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
	icp.setMaxCorrespondenceDistance (115);
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);
//	pcl::PointCloud<pcl::PointXYZRGBA> Final;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Final(new pcl::PointCloud<pcl::PointXYZRGBA>);
	icp.align(*Final);
	*Final +=*cloud_out;
	ShowCloud(Final,0);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return (0);
}
