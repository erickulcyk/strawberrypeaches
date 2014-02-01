#include "RenderWindowUIMultipleInheritance.h"
#include <QFileDialog>
#include <cstdio>
#include <boost/filesystem/path.hpp>

#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>
#include <vtkSmartPointer.h>
#include <pcl/common/common_headers.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/io/vtk_lib_io.h>
#include <vtkPointPicker.h>

#include <string>

using namespace std;

const string RenderWindowUIMultipleInheritance::pcd = ".pcd";
const string RenderWindowUIMultipleInheritance::ply = ".ply";
const string RenderWindowUIMultipleInheritance::stl = ".stl";
const string RenderWindowUIMultipleInheritance::proj = ".spp";

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
		
void GetPoints(string name, int index,double radius);

typedef Eigen::Matrix< double, 4, 1> vector4d;
std::string selectedItem;
int previousIndex;
double previousRadius;
string previousCloudName;

void TransformPointCloud(PointCloud &pointCloud, float scale, float x, float y, float z, vector4d& location);

void TransformPolygonMesh(pcl::PolygonMesh::Ptr &polyMesh, float scale, float x, float y, float z, vector4d& location);

void TransformSelectedItem(float scale, float x, float y, float z);

void GetNewCloud(string oldName, PointCloud::Ptr newCloud, set<int>& indices);
void GetSetInfo(string cloudName, set<int>& indices, double center[3], double minPt[3], double maxPt[3]);

pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("viz", false)); 
PointCloud::Ptr clicked_points_3d (new PointCloud);

std::map<std::string, pcl::PolygonMesh::Ptr> objects;
std::map<std::string, PointCloud::Ptr> clouds;
std::map<std::string, vector4d > locations;

std::set<int> pointsToRemove;

const int LEFT = 0;
const int RIGHT = 1;
const int UP = 2;
const int DOWN = 3;
const int ZOOM_IN = 4;
const int ZOOM_OUT = 5;
const int RESET = 6;
const int DS = 7;
const int FORWARD = 8;
const int BACKWARD = 9;
const int LEFT_TURN = 10;
const int RIGHT_TURN = 11;

struct args_a
{
	// structure used to pass arguments to the callback function
	PointCloud::Ptr clicked_points;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void RotateGeneral(double in[3], double axis[3], double a, double r[3])
{
	double x = in[0], y = in[1], z = in[2];
	double u = axis[0], v = axis[1], w = axis[2];

	double m1 = (u*x+v*y+w*z)*(1-cos(a));
	double len = ( u*u + v*v + w*w );

	r[0] = u*m1 + x*cos(a)+(-w*y+v*z)*sin(a);
	r[0] /= len;

	r[1] = v*m1 + y*cos(a)+(w*x-u*z)*sin(a);
	r[1] /= len;

	r[2] = w*m1 + z*cos(a)+(-v*x+u*y)*sin(a);
	r[2] /= len;

}

void moveCamera(bool rotate, int dir, void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

	std::vector<pcl::visualization::Camera> camera;
	camera.resize(5);

	double scale = 10;

	viewer->getCameras(camera);
	double viewX = camera[0].view[0];
	double viewY = camera[0].view[1];
	double viewZ = camera[0].view[2];
	double posX = camera[0].pos[0];
	double posY = camera[0].pos[1];
	double posZ = camera[0].pos[2];
	double lookAtX = camera[0].focal[0];
	double lookAtY = camera[0].focal[1];
	double lookAtZ = camera[0].focal[2];

	cout<<" Focal point: "<<lookAtX<<" "<<lookAtY<<" "<<lookAtZ<<endl;
	cout<<"Pos: "<<posX<<" "<<posY<<" "<<posZ<<endl;

	double viewDir[3];
	double len = 0;
	for(int i =0; i<3; i++)
	{
		viewDir[i] = camera[0].focal[i]-camera[0].pos[i];
		len += viewDir[i]*viewDir[i];
	}

	len = sqrt(len);
	cout<<"View Dir: ";
	for(int i =0; i<3; i++)
	{
		viewDir[i]/=len;
		cout<<viewDir[i]<<" ";
	}

	cout<<"Up: "<<viewX<<" "<<viewY<<" "<<viewZ<<endl;

	double crossProduct[3];
	crossProduct[0] = (viewY*viewDir[2] - viewZ*viewDir[1])/scale;
	crossProduct[1] = (viewZ*viewDir[0] - viewX*viewDir[2])/scale;
	crossProduct[2] = (viewX*viewDir[1] - viewY*viewDir[0])/scale;

	double angle = 10.0/180.0*3.141592635;
	double up[3];
	up[0] = viewX;
	up[1] = viewY;
	up[2] = viewZ;

	cout<<"CP: "<<crossProduct[0]<<" "<<crossProduct[1]<<" "<<crossProduct[2]<<endl;

	cout<<endl;
	cout<<"Clip: "<<camera[0].clip[0]<<" "<<camera[0].clip[1]<<endl;
	viewer -> setCameraClipDistances(0,100,0);
	if(!rotate){
		//move
		switch (dir)
		{
			case UP:
				viewer -> setCameraPosition(
					posX + viewX/scale,
					posY + viewY/scale,
					posZ + viewZ/scale,
					lookAtX + viewX/scale,
					lookAtY + viewY/scale,
					lookAtZ + viewZ/scale,
					viewX,
					viewY, 
					viewZ,
					0);
				break;
			case DOWN:
				viewer -> setCameraPosition(
					posX - viewX/scale,
					posY - viewY/scale,
					posZ - viewZ/scale,
					lookAtX - viewX/scale,
					lookAtY - viewY/scale,
					lookAtZ - viewZ/scale,
					viewX,
					viewY, 
					viewZ,
					0);
				break;
			case RESET:
				viewer -> setCameraPosition(0, 0, 0, 0, 0, 1, 0, 1, 0, 0);
				break;
			case FORWARD:
				viewer -> setCameraPosition(
					posX + viewDir[0],
					posY + viewDir[1],
					posZ + viewDir[2],
					lookAtX + viewDir[0],
					lookAtY + viewDir[1],
					lookAtZ + viewDir[2],
					viewX,
					viewY, 
					viewZ,
					0);
				break;
			case BACKWARD:
				viewer -> setCameraPosition(
					posX - viewDir[0],
					posY - viewDir[1],
					posZ - viewDir[2],
					lookAtX - viewDir[0],
					lookAtY - viewDir[1],
					lookAtZ - viewDir[2],
					viewX,
					viewY, 
					viewZ,
					0);
				break;
			case LEFT:
				viewer -> setCameraPosition(
					posX + crossProduct[0],
					posY + crossProduct[1],
					posZ + crossProduct[2],
					lookAtX + crossProduct[0],
					lookAtY + crossProduct[1],
					lookAtZ + crossProduct[2],
					viewX,
					viewY, 
					viewZ,
					0);
				break;
			case RIGHT:
				viewer -> setCameraPosition(
					posX - crossProduct[0],
					posY - crossProduct[1],
					posZ - crossProduct[2],
					lookAtX - crossProduct[0],
					lookAtY - crossProduct[1],
					lookAtZ - crossProduct[2],
					viewX,
					viewY, 
					viewZ,
					0);
				break;
			case LEFT_TURN:
				double lookLeft[3];
				RotateGeneral(up, viewDir, 5.0/180.0*3.14159, lookLeft);
				viewer -> setCameraPosition(
					posX, 
					posY,
					posZ,
					lookAtX,
					lookAtY,
					lookAtZ,
					lookLeft[0],
					lookLeft[1], 
					lookLeft[2],
					0);
				break;
			case RIGHT_TURN:
				double lookRight[3];
				RotateGeneral(up, viewDir, -5.0/180.0*3.14159, lookRight);
				viewer -> setCameraPosition(
					posX, 
					posY,
					posZ,
					lookAtX,
					lookAtY,
					lookAtZ,
					lookRight[0],
					lookRight[1], 
					lookRight[2],
					0);
				break;
		}
	}
	else {
		//rotate
	}
}

void pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
//	args_a* data = (struct args_a *)args;
	cout<<"In point callback method"<<endl;
	if (event.getPointIndex () == -1)
		return;
	cout<<"Something selected!"<<endl;

	cout<<"Point Index: "<<event.getPointIndex()<<endl;
	cout<<"Clouds size: "<<clouds.size()<<endl;
	string fcloud = clouds.begin()->first;
	GetPoints(fcloud, event.getPointIndex(),.1);

	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	clicked_points_3d->clear();
	clicked_points_3d->push_back(current_point);
	// Draw clicked points in red:
	viewer->removePointCloud("clicked_points");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red (clicked_points_3d, 255, 255, 255);
	viewer->addPointCloud(clicked_points_3d, red, "clicked_points");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	//viewer->updatePointCloud(clicked_points_3d,"clicked_points");
	
	std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

void SetupPointCloud
(
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr
)
{
	std::cout << "Genarating example point clouds.\n\n";
	// We're going to make an ellipse extruded along the z-axis. The colour for
	// the XYZRGB cloud will gradually go from red to green to blue.
	uint8_t r(255), g(15), b(15);
	for (float z(-1.0); z <= 1.0; z += 0.05)
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
		{
			pcl::PointXYZ basic_point;
			basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
			basic_point.y = sinf (pcl::deg2rad(angle));
			basic_point.z = z;
			basic_cloud_ptr->points.push_back(basic_point);

			pcl::PointXYZRGB point;
			point.x = basic_point.x;
			point.y = basic_point.y;
			point.z = basic_point.z;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
					static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back (point);
		}

		if (z < 0.0)
		{
			r -= 12;
			g += 12;
		}
		else
		{
			g -= 12;
			b += 12;
		}
	}

	basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
	point_cloud_ptr->height = 1;
}

void RenderWindowUIMultipleInheritance::AddViewerToWidget()
{
	vtkSmartPointer<vtkRendererCollection> renderCollection =
		viewer->getRendererCollection();
	for(vtkRenderer* i = renderCollection->GetFirstRenderer(); i!=NULL; i=renderCollection->GetNextItem())
	{
		this->qvtkWidget->GetRenderWindow()->AddRenderer(i);
	}

	vtkSmartPointer<vtkRenderWindowInteractor> interactor = this->qvtkWidget->GetRenderWindow()->GetInteractor();
	interactor->SetInteractorStyle(viewer->getInteractorStyle());
	vtkSmartPointer<vtkPointPicker> pp = vtkSmartPointer<vtkPointPicker>::New ();
  	pp->SetTolerance (pp->GetTolerance () * 2);
    	interactor->SetPicker (pp);
//	viewer->SetInteractor(this->qvtkWidget->GetRenderWindow()->GetInteractor());
}

void RenderWindowUIMultipleInheritance::ReplaceWithObject(string path, string fileName, string ext)
{
	if(ext==stl)
	{
		pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
		pcl::io::loadPolygonFileSTL (path, *mesh);
		objects[fileName] = mesh;

		pcl::PointCloud<PointT> cloud2;
		pcl::fromPCLPointCloud2(mesh->cloud, cloud2);
		vector4d centroid;
		pcl::compute3DCentroid(cloud2, centroid);

		double center[3], minPt[3], maxPt[3];
		GetSetInfo(previousCloudName, pointsToRemove, center, minPt, maxPt);
		double dist = sqrt(
			(maxPt[0]-minPt[0])*(maxPt[0]-minPt[0])+
			(maxPt[1]-minPt[1])*(maxPt[1]-minPt[1])+
			(maxPt[2]-minPt[2])*(maxPt[2]-minPt[2]));
		PointT pmin, pmax;
		double actualLength = pcl::getMaxSegment(cloud2, pmin, pmax);

		viewer->addPolygonMesh(*mesh, fileName);
		TransformPolygonMesh(mesh, dist/actualLength, center[0]-centroid[0], center[1] - centroid[1], center[2] - centroid[2], centroid);
		viewer->updatePolygonMesh(*mesh, fileName);
		pcl::fromPCLPointCloud2(mesh->cloud, cloud2);
		pcl::compute3DCentroid(cloud2, centroid);
		locations[fileName] = centroid;
	}
	else if (ext==pcd)
	{
		PointCloud::Ptr cloud(new PointCloud);	
		pcl::io::loadPCDFile (path, *cloud);
		//EuclideanCluster(cloud);
		int index = 0;
		clouds[fileName] = cloud;
		viewer->addPointCloud(cloud,fileName);

		vector4d centroid;
		pcl::compute3DCentroid(*cloud, centroid);
		locations[fileName] = centroid;
		cout<<"Got centroid: "<<centroid[0]<<" "<<centroid[1]<<" "<<centroid[2]<<endl;

	}

	selectedItem = fileName;
}

void RenderWindowUIMultipleInheritance::AddObject(string path, string fileName, string ext)
{
	if(ext==stl)
	{
		pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
		pcl::io::loadPolygonFileSTL (path, *mesh);
		viewer->addPolygonMesh(*mesh, fileName);
		objects[fileName] = mesh;

		pcl::PointCloud<PointT> cloud2;
		pcl::fromPCLPointCloud2(mesh->cloud, cloud2);
		vector4d centroid;
		pcl::compute3DCentroid(cloud2, centroid);
		locations[fileName] = centroid;
	}
	else if (ext==pcd)
	{
		PointCloud::Ptr cloud(new PointCloud);	
		pcl::io::loadPCDFile (path, *cloud);
		//EuclideanCluster(cloud);
		int index = 0;
		clouds[fileName] = cloud;
		viewer->addPointCloud(cloud,fileName);

		vector4d centroid;
		pcl::compute3DCentroid(*cloud, centroid);
		locations[fileName] = centroid;
		cout<<"Got centroid: "<<centroid[0]<<" "<<centroid[1]<<" "<<centroid[2]<<endl;

	}

	selectedItem = fileName;
}

void AddTreeItem(QTreeWidgetItem* parent, string name, bool expand, bool select)
{
	QTreeWidgetItem* child = new QTreeWidgetItem(parent);
	child->setText(0,QString::fromStdString(name));
	if(expand)
	{
		parent->setExpanded(true);
	}

	if(select)
	{
		child->setSelected(true);
	}	
}

void GetPoints(string name, int index, double radius)
{
	previousIndex = index;
	previousRadius = radius;
	previousCloudName = name;

	PointCloud::Ptr cloud = clouds[name];

	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud);
	std::vector<int> k_indices;
	std::vector<float> k_sqr_distances;

	tree->radiusSearch
	(
		cloud->points[index],
		radius,
		k_indices,
		k_sqr_distances,
		0
	);

	cout<<"Number of Points: "<<k_indices.size()<<endl;
	pointsToRemove.insert(index);
	for(int i = 0; i < k_indices.size(); i++)
	{
		int ind = k_indices[i];
		cloud->points[ind].r = 255;
		cloud->points[ind].g = 255;
		cloud->points[ind].b = 255;
		
		pointsToRemove.insert(ind);
	}

	viewer->updatePointCloud(cloud,name);
}

void GetNewCloud(string oldName, PointCloud::Ptr newCloud, set<int>& indices)
{
	PointCloud::Ptr old = clouds[oldName];
	int j = 0;
	for(set<int>::iterator it = indices.begin(); it!=indices.end(); it++)
	{
		int index = *it;
		while(j<index)
		{
			newCloud->push_back(old->points[j]);
			j++;
		}

		j++;
	}
}

void GetSetInfo(string cloudName, set<int>& indices, double center[3], double minPt[3], double maxPt[3])
{
	center[0] = 0;
	center[1] = 1;
	center[2] = 2;

	PointCloud::Ptr cloud = clouds[cloudName];
	bool first = true;
	for(set<int>::iterator it = indices.begin(); it!=indices.end(); it++)
	{
		PointT pt = cloud->points[*it];
		center[0] += pt.x;
		center[1] += pt.y;
		center[2] += pt.z;

		double x = pt.x;
		double y = pt.y;
		double z = pt.z;

		if(first)
		{
			minPt[0] = x;
			minPt[1] = y;
			minPt[2] = z;

			maxPt[0] = x;
			maxPt[1] = y;
			maxPt[2] = z;
		}
		else
		{
			minPt[0] = min(minPt[0], x);
			minPt[1] = min(minPt[1], y);
			minPt[2] = min(minPt[2], z);

			maxPt[0] = max(maxPt[0], x);
			maxPt[1] = max(maxPt[1], y);
			maxPt[2] = max(maxPt[2], z);
		}

		first = false;
	}

	center[0]/=indices.size();
	center[1]/=indices.size();
	center[2]/=indices.size();
}

void RenderWindowUIMultipleInheritance::EuclideanCluster(PointCloud::Ptr cloud_filtered)
{
	PointCloud::Ptr  cloud_f (new PointCloud);

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	PointCloud::Ptr cloud_plane (new PointCloud);

	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.02);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.025); // 2cm
	ec.setMinClusterSize (1);
	//ec.setMaxClusterSize (0);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);
/*
	int i=0, nr_points = (int) cloud_filtered->points.size ();
	while (cloud_filtered->points.size () > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Get the points associated with the planar surface
		extract.filter (*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud_filtered = *cloud_f;
	}
*/
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		PointCloud::Ptr cloud_cluster (new PointCloud);
		int index = 0;
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		{
			index++;
			if(j==0)
			{
				cloud_filtered->points[*pit].r = 255;
				cloud_filtered->points[*pit].g = 0;
				cloud_filtered->points[*pit].b = 0;
			}
			else if(j==1)
			{
				cloud_filtered->points[*pit].r = 0;
				cloud_filtered->points[*pit].g = 255;
				cloud_filtered->points[*pit].b = 0;
			}
			else if(j==2)
			{
				cloud_filtered->points[*pit].r = 0;
				cloud_filtered->points[*pit].g = 0;
				cloud_filtered->points[*pit].b = 255;
			}

			cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
		}


		cout<<"Index: "<<index<<endl;

		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		j++;
	}
}

void TransformSelectedItem(float scale, float x, float y, float z)
{
	cout<<"Selected item: "<<selectedItem<< " "<<clouds.size()<<" "<<objects.size()<<endl;
	if(clouds.find(selectedItem)!=clouds.end())
	{
		vector4d& loc = locations[selectedItem];
		PointCloud::Ptr pointCloud = clouds[selectedItem];
		TransformPointCloud(*pointCloud, scale, x, y, z, loc);
		viewer->updatePointCloud(pointCloud, selectedItem);
	}
	else if(objects.find(selectedItem)!=objects.end())
	{
		vector4d loc = locations[selectedItem];
		pcl::PolygonMesh::Ptr a = objects[selectedItem];
		TransformPolygonMesh(a, scale, x, y, z, loc);
		viewer->updatePolygonMesh(*a, selectedItem);
	}
}

void Rotz(pcl::PolygonMesh::Ptr &polyMesh, float z)
{
	pcl::PointCloud<pcl::PointXYZ> cloud2;
	pcl::fromPCLPointCloud2(polyMesh->cloud, cloud2);

	vector4d centroid;
	pcl::compute3DCentroid(cloud2, centroid);

	Eigen::Matrix4f sub;
	sub<<
		1, 		0,			.0,		-centroid[0],
		0, 		1,			.0,		-centroid[1],
		.0,		.0,			1.0,	-centroid[2],
		.0,     .0,     	.0,		1.0;
	Eigen::Matrix4f add;
	add<<
		1, 		0,			.0,		centroid[0],
		0, 		1,			.0,		centroid[1],
		.0,		.0,			1.0,	centroid[2],
		.0,     .0,     	.0,		1.0;

	Eigen::Matrix4f a;
	a<<
		cos(z), -sin(z),	.0,		0,
		sin(z), cos(z),		.0,		0,
		.0,		.0,			1.0,	0,
		.0,     .0,     	.0,		1.0;

	pcl::transformPointCloud(cloud2,cloud2, sub);
	pcl::transformPointCloud(cloud2,cloud2, a);
	pcl::transformPointCloud(cloud2,cloud2, add);

	pcl::toPCLPointCloud2(cloud2, polyMesh->cloud);
}

void Roty(pcl::PolygonMesh::Ptr &polyMesh, float y)
{
	pcl::PointCloud<pcl::PointXYZ> cloud2;
	pcl::fromPCLPointCloud2(polyMesh->cloud, cloud2);

	vector4d centroid;
	pcl::compute3DCentroid(cloud2, centroid);

	Eigen::Matrix4f sub;
	sub<<
		1, 		0,			.0,		-centroid[0],
		0, 		1,			.0,		-centroid[1],
		.0,		.0,			1.0,	-centroid[2],
		.0,     .0,     	.0,		1.0;
	Eigen::Matrix4f add;
	add<<
		1, 		0,			.0,		centroid[0],
		0, 		1,			.0,		centroid[1],
		.0,		.0,			1.0,	centroid[2],
		.0,     .0,     	.0,		1.0;

	Eigen::Matrix4f a;
	a<<
		 cos(y), 	.0,     sin(y),	0,
		.0,     	1.0,	0,		0,
		-sin(y),	.0,		cos(y),	0,
		.0,     	.0,     .0,		1.0;

	pcl::transformPointCloud(cloud2,cloud2, sub);
	pcl::transformPointCloud(cloud2,cloud2, a);
	pcl::transformPointCloud(cloud2,cloud2, add);
	pcl::toPCLPointCloud2(cloud2, polyMesh->cloud);
}

void Rotx(pcl::PolygonMesh::Ptr &polyMesh, float x)
{
	pcl::PointCloud<pcl::PointXYZ> cloud2;
	pcl::fromPCLPointCloud2(polyMesh->cloud, cloud2);

	vector4d centroid;
	pcl::compute3DCentroid(cloud2, centroid);

	Eigen::Matrix4f sub;
	sub<<
		1, 		0,			.0,		-centroid[0],
		0, 		1,			.0,		-centroid[1],
		.0,		.0,			1.0,	-centroid[2],
		.0,     .0,     	.0,		1.0;
	Eigen::Matrix4f add;
	add<<
		1, 		0,			.0,		centroid[0],
		0, 		1,			.0,		centroid[1],
		.0,		.0,			1.0,	centroid[2],
		.0,     .0,     	.0,		1.0;

	Eigen::Matrix4f a;
	a<<
		 1, 	.0,     .0,		0,
		.0,     cos(x),	-sin(x),	0,
		.0,     sin(x),	cos(x),		0,
		.0,     .0,     .0,	     	1.0;

	pcl::transformPointCloud(cloud2,cloud2, sub);
	pcl::transformPointCloud(cloud2,cloud2, a);
	pcl::transformPointCloud(cloud2,cloud2, add);
	pcl::toPCLPointCloud2(cloud2, polyMesh->cloud);
}

void TransformPolygonMesh(pcl::PolygonMesh::Ptr &polyMesh, float scale, float x, float y, float z, vector4d& location)
{
	pcl::PointCloud<PointT> cloud2;
	pcl::fromPCLPointCloud2(polyMesh->cloud, cloud2);
	TransformPointCloud(cloud2, scale, x, y, z, location);
	pcl::toPCLPointCloud2(cloud2, polyMesh->cloud);
}

void TransformPointCloud(PointCloud &pointCloud, float scale, float x, float y, float z, vector4d& location)
{
	Eigen::Matrix4f a;
	float offset = scale-1;
	a<<
		scale,  .0,     .0,     x - location[0]*offset,
		.0,     scale,  .0,     y - location[1]*offset,
		.0,     .0,     scale,  z - location[2]*offset,
		.0,     .0,     .0,     1.0;

	location[0]+=x;
	location[1]+=y;
	location[2]+=z;

	pcl::transformPointCloud(pointCloud,pointCloud, a);
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* window_void) 
{
	if(!event.keyDown())
	{
		return;
	}

	cout<<"Key pressed: "<<event.getKeySym()<<endl;
	
	RenderWindowUIMultipleInheritance* window = static_cast<RenderWindowUIMultipleInheritance*> (window_void);
	pcl::PolygonMesh::Ptr a = objects[selectedItem];
	string key = event.getKeySym();
	if(key == "a")
		moveCamera(false, LEFT, &viewer);
	else if (key == "d")
		moveCamera(false, RIGHT, (void*)&viewer);
	else if (key == "w")
		moveCamera(false, FORWARD, (void*)&viewer); //UP
	else if (key == "s")
		moveCamera(false, BACKWARD, (void*)&viewer);
	else if (key == "e")
		moveCamera(false, UP, (void*)&viewer);
	else if (key == "x")
		moveCamera(false, DOWN, (void*)&viewer);
	else if (key == "t")
		moveCamera(false, LEFT_TURN, (void*)&viewer);
	else if (key == "b")
		moveCamera(false, RIGHT_TURN, (void*)&viewer);
	else if (key == "f")
		moveCamera(false, DS, (void*)&viewer);
	else if (key == "quoteleft")
		GetPoints(previousCloudName, previousIndex, previousRadius + .05);
	else if (key == "asciitilde")
	{
		PointCloud::Ptr newCloud (new PointCloud);
		GetNewCloud(previousCloudName, newCloud, pointsToRemove);
		viewer->updatePointCloud(newCloud,previousCloudName);
		clouds[previousCloudName] = newCloud;

		clicked_points_3d->clear();
		viewer->updatePointCloud(clicked_points_3d, "clicked_points");
	}
	else if (key == "bracketright") //Bigger
		TransformSelectedItem(1.1,0,0,0);
	else if (key == "bracketleft") //smaller
		TransformSelectedItem(.9,0,0,0);
	else if (key == "Up")
		TransformSelectedItem(1,0,.1,0);
	else if (key == "Down")
		TransformSelectedItem(1,0,-.1,0);
	else if (key == "Right")
		TransformSelectedItem(1,.1,0,0);
	else if (key == "Left")
		TransformSelectedItem(1,-.1,0,0);
	else if (key == "comma")
		TransformSelectedItem(1,0,0,-.1);
	else if (key == "period")
		TransformSelectedItem(1,0,0,.1);
	else if (key == "j")
	{
		Rotx(a, .1);
		viewer->updatePolygonMesh(*a, selectedItem);
	}
	else if (key == "k")
	{
		Roty(a, .1);
		viewer->updatePolygonMesh(*a, selectedItem);
	}
	else if (key == "l")
	{
		Rotz(a, .1);
		viewer->updatePolygonMesh(*a, selectedItem);
	}
	else if (key == "y")
	{
		Rotx(a, -.1);
		viewer->updatePolygonMesh(*a, selectedItem);
	}
	else if (key == "u")
	{
		Roty(a, -.1);
		viewer->updatePolygonMesh(*a, selectedItem);
	}
	else if (key == "i")
	{
		Rotz(a, -.1);
		viewer->updatePolygonMesh(*a, selectedItem);
	}
	
	window->render();
}

void RenderWindowUIMultipleInheritance::RenderForwards()
{
	//this->qvtkWidget->GetInteractor()->zoomOut();
}

void RenderWindowUIMultipleInheritance::render()
{
	QVTKInteractor*	Interactor = this->qvtkWidget->GetInteractor();
	double factor = 10.0 * 0.2 * .5;
	Interactor->SetDolly (pow (1.1, factor));

	vtkRenderer* a = Interactor->FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);
	a->Render();

	vtkSmartPointer<vtkRendererCollection> renderCollection = viewer->getRendererCollection();
	for(vtkRenderer* i = renderCollection->GetFirstRenderer(); i!=NULL; i=renderCollection->GetNextItem())
	{
		i->Render();
	}

	this->qvtkWidget->GetRenderWindow()->Render();
	this->qvtkWidget->GetInteractor()->Render();

}

void RenderWindowUIMultipleInheritance::on_treeWidget_itemActivated(QTreeWidgetItem *item, int column)
{
	selectedItem = item->text(column).toUtf8().constData();
	cout<<"Selected Item: "<<selectedItem<<endl;
}

void RenderWindowUIMultipleInheritance::on_actionReplace_triggered()
{
	QString fileName = QFileDialog::getOpenFileName
	(
		this,
		tr("Open File"),
		"",
		tr("Files (*.*)")
	);
	
	if(fileName.isEmpty())
		return;
	string fn = fileName.toUtf8().constData();	
	boost::filesystem::path fnPath(fn);
	if(!fnPath.has_extension() || !fnPath.has_filename())
		return;
	string ext = fnPath.extension().string();

	string widgetText;
	if(ext == pcd)
		widgetText = "Point Clouds";
	else if (ext==ply || ext==stl)
		widgetText = "3D Objects";

	QTreeWidget *tree = this->treeWidget;
	QList<QTreeWidgetItem *> pcItems = tree->findItems
	(
		QString::fromStdString(widgetText),
		Qt::MatchExactly,
		0
	);

	if(!pcItems.empty())
	{
		QTreeWidgetItem* pcItem = pcItems.at(0);
		string file_name = fnPath.filename().string();
		AddTreeItem (pcItem, file_name, true, true);
		ReplaceWithObject(fn, file_name,ext);
	}
}

void RenderWindowUIMultipleInheritance::on_actionImport_triggered()
{
	QString fileName = QFileDialog::getOpenFileName
	(
		this,
		tr("Open File"),
		"",
		tr("Files (*.*)")
	);
	
	if(fileName.isEmpty())
		return;
	string fn = fileName.toUtf8().constData();	
	boost::filesystem::path fnPath(fn);
	if(!fnPath.has_extension() || !fnPath.has_filename())
		return;
	string ext = fnPath.extension().string();

	string widgetText;
	if(ext == pcd)
		widgetText = "Point Clouds";
	else if (ext==ply || ext==stl)
		widgetText = "3D Objects";

	QTreeWidget *tree = this->treeWidget;
	QList<QTreeWidgetItem *> pcItems = tree->findItems
	(
		QString::fromStdString(widgetText),
		Qt::MatchExactly,
		0
	);

	if(!pcItems.empty())
	{
		QTreeWidgetItem* pcItem = pcItems.at(0);
		string file_name = fnPath.filename().string();
		AddTreeItem (pcItem, file_name, true, true);
		AddObject(fn, file_name,ext);
	}
}

void RenderWindowUIMultipleInheritance::SetupViewer()
{
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red (clicked_points_3d, 255, 255, 255);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	viewer->addPointCloud(clicked_points_3d, red, "clicked_points");

	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)this);
	//struct args_a cb_args;
	//cb_args.clicked_points = clicked_points_3d;
	//cb_args.viewerPtr = viewer;

	viewer->registerPointPickingCallback(pp_callback);//, (void*)&cb_args);
/*	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	SetupPointCloud(basic_cloud_ptr, point_cloud_ptr);
*/
	viewer->setBackgroundColor (0, 0, 0);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
//	viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "sample cloud");
//	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
}

// Constructor
RenderWindowUIMultipleInheritance::RenderWindowUIMultipleInheritance()
{
	this->setupUi(this);

	SetupViewer();
	AddViewerToWidget();

	connect(this->actionExit, SIGNAL(triggered()), this, SLOT(slotExit()));

	this->qvtkWidget->setFocus();
};

void RenderWindowUIMultipleInheritance::slotExit()
{
	qApp->exit();
}
