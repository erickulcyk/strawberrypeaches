#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <fstream>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <sstream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/PCLPointCloud2.h>//1.7 only

using namespace std;

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

void ReadCloud (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, ifstream& file, int index, bool useBGR)
{
	char dsize[4];
	file.read (dsize, 4);
	cout<<"READ: "<<(int)(dsize[0])<<" "<<(int)(dsize[1])<<" "<<(int)(dsize[2])<<" "<<(int)(dsize[3])<<endl;
	ifstream::pos_type size = (unsigned char)dsize[0];
	size = size<<8;
	size += (unsigned char)dsize[1];
	size = size<<8;
	size += (unsigned char)dsize[2];
	size = size<<8;
	size += (unsigned char)dsize[3];
	cout<<"Got Size: "<<size<<" "<<endl;
	char* memblock = new char [10*size];
	file.read (memblock, 10*size);
	unsigned char* umem = reinterpret_cast<unsigned char*>(memblock);

	cloud->width    = size;
	cloud->height   = 1;
	cloud->is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
			cloud->points[i].x = (signed short)(((umem[i*10+0])<<8) + umem[i*10+1]);
			cloud->points[i].y = (signed short)(((umem[i*10+2])<<8) + umem[i*10+3]);
			cloud->points[i].z = -(signed short)(((umem[i*10+4])<<8) + umem[i*10+5]);
			cloud->points[i].x/=1000;
			cloud->points[i].y/=1000;
			cloud->points[i].z/=1000;

			if(i<5 || i+5>=cloud->points.size())
			{
				cout<<"Point: "<<i<<" "<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z<<endl;
			}
		if(!useBGR)
		{
			
			cloud->points[i].b = memblock[i*10+6];
			cloud->points[i].g = memblock[i*10+7];
			cloud->points[i].r = memblock[i*10+8];
			cloud->points[i].a = memblock[i*10+9];
		}
		else
		{
			if(index%3==0)
			{
				cloud->points[i].b = 255;
				cloud->points[i].g = 0;
				cloud->points[i].r = 0;
				cloud->points[i].a = 0;
			}
			else if (index%3==1)
			{
				cloud->points[i].b = 0;
				cloud->points[i].g = 255;
				cloud->points[i].r = 0;
				cloud->points[i].a = 0;

			}
			else
			{
				cloud->points[i].b = 0;
				cloud->points[i].g = 0;
				cloud->points[i].r = 255;
				cloud->points[i].a = 0;
			}
		}
	}

	cout<<"Set cloud"<<endl;	

	delete[] memblock;
}

void SaveToPly(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, string name)
{
	pcl::PLYWriter plyWriter;
	plyWriter.write(name, *cloud);
}

void MakePolygons(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	pcl::OrganizedFastMesh<pcl::PointXYZRGBA> ofm;
	ofm.setInputCloud(cloud);
	pcl::PolygonMesh polymesh;
	ofm.reconstruct(polymesh);
	cout<<"Done constructing mesh"<<endl;

	//pcl::io::savePolygonFileSTL("sample2.stl",polymesh); 
/*
	pcl::PLYWriter plyWriter;
	plyWriter.writeASCII("sample.ply",cloud);*/
	/*
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	// normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	// cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();*/
}

void FilterCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	sensor_msgs::PointCloud2::Ptr msg (new sensor_msgs::PointCloud2);
	sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2);
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>); 
	pcl::toROSMsg (*cloud, *msg);

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
		<< endl;

	// Create the filtering object
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (msg);
	sor.setLeafSize (10.0f, 10.0f, 10.0f);
	sor.filter (*cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
		<< endl;
}

int main (int argc, char** argv)
{
	cout<<"Usage: [# iterations], [Use rgb], [ply file name], [use ICP]"<<endl;
	int iters = 3;
	if(argc>1)
	{
		sscanf(argv[1],"%d",&iters);
		cout<<"Iters: "<<iters<<endl;
	}

	bool useBGR = 1;
	int a;
	if(argc>2)
	{
		sscanf(argv[2],"%d",&a);
		useBGR = a!=0;
	}

	string plyFile = "sample2.ply";
	if(argc>3)
	{
		plyFile = string(argv[3]);
	}

	bool useICP = false;
	if(argc>4)
	{
		sscanf(argv[4],"%d",&a);
		useICP = a!=0;
	}

	bool savePly = true;
	if(argc>5)
	{
		sscanf(argv[5],"%d",&a);
		savePly = a!=0;
	}

	//  pcl::PointCloud<pcl::PointXYZRGBA> cloud, cloud2;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>); 
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>); 
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZRGBA>); 



	// Fill in the cloud data

	cloud2->width    = 1000;
	cloud2->height   = 1;
	cloud2->is_dense = false;
	cloud2->points.resize (cloud2->width * cloud2->height);

	ifstream file ("C:\\Users\\Eric\\Documents\\Visual Studio 2013\\Projects\\KinectExplorer-WPF\\bin\\Debug\\KinectDepth2.txt", ios::in|ios::binary|ios::ate);
	if (file.is_open())
	{
		file.seekg (0, ios::beg);
		for(int  i=0; i <iters; i++)
		{
			if(i==0)
			{
				ReadCloud(cloud, file,i, useBGR);
				ReadCloud(cloud, file,i, useBGR);
				ReadCloud(cloud, file,i, useBGR);
				ReadCloud(cloud, file,i, useBGR);
				ShowCloud(cloud,i);
				*cloud3+=*cloud;

			//	FilterCloud(cloud3);
				//MakePolygons(cloud);
			}
			else if(i%2==1)
			{
				ReadCloud(cloud2, file,i, useBGR);
				ReadCloud(cloud2, file,i, useBGR);
				ReadCloud(cloud2, file,i, useBGR);
				ReadCloud(cloud2, file,i, useBGR);
				ShowCloud(cloud2,i);
				if(useICP)
				{
					pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;

					icp.setInputCloud (cloud2);
					icp.setInputTarget(cloud);
					icp.setMaximumIterations (2);
					icp.setMaxCorrespondenceDistance (1);

					cout<<"Trying to align!"<<endl;
					icp.align(*cloud2);
					Eigen::Matrix4f transformation = icp.getFinalTransformation ();

					cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;


					cout << transformation << endl;
					cout<<"Size afer icp: "<<cloud2->size()<<" "<<cloud3->size()<<endl;
				}

				*cloud3+=*cloud2;
			}
			else if(i%2==0)
			{
				ReadCloud(cloud, file,i, useBGR);
				ReadCloud(cloud, file,i, useBGR);
				ReadCloud(cloud, file,i, useBGR);
				ReadCloud(cloud, file,i, useBGR);
				ShowCloud(cloud,i);

				if(useICP)
				{
					pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;

					icp.setInputCloud (cloud);
					icp.setInputTarget(cloud2);
					icp.setMaximumIterations (2);
					icp.setMaxCorrespondenceDistance (1);

					cout<<"Trying to align!"<<endl;
					icp.align(*cloud);
					Eigen::Matrix4f transformation = icp.getFinalTransformation ();

					cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;


					cout << transformation << endl;
					cout<<"Size afer icp: "<<cloud->size()<<" "<<cloud3->size()<<endl;
				}
				
				*cloud3+=*cloud;
			}
		}

		cout<<"FINAL CLOUD!"<<endl;
		ShowCloud(cloud3,-1);
		cout<<"Saving to ply file..."<<endl;
		if(savePly)
			SaveToPly(cloud3, plyFile);
		//MakePolygons(cloud3);
	}
	else cout << "Unable to open file";

	std::cout<<"DONE!\n";
	/*
	   pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
	   std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

	   for (size_t i = 0; i < cloud.points.size (); ++i)
	   std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
	   */
	return (0);
}

