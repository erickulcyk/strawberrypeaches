#ifndef RenderWindowUIMultipleInheritance_H
#define RenderWindowUIMultipleInheritance_H

#include "ui_RenderWindowUIMultipleInheritance.h"
#include <cstdio>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>
#include <map>
#include<QMainWindow>
#include<QMessageBox>
#include<QKeyEvent>
#include <QMainWindow>
#include <pcl/visualization/pcl_visualizer.h>

class RenderWindowUIMultipleInheritance : public QMainWindow, private Ui::RenderWindowUIMultipleInheritance
{
	Q_OBJECT
	public:

		RenderWindowUIMultipleInheritance();
		//void keyPressEvent( QKeyEvent *event );
		
		void render();
		void RenderForwards();

		public slots:

			virtual void slotExit();
	
	private:
		typedef pcl::PointXYZRGBA PointT;
		typedef pcl::PointCloud<PointT> PointCloud;

		void Rotx(pcl::PolygonMesh::Ptr &polyMesh, float x);
		void Roty(pcl::PolygonMesh::Ptr &polyMesh, float y);
		void Rotz(pcl::PolygonMesh::Ptr &polyMesh, float z);


		void EuclideanCluster(PointCloud::Ptr cloud_filtered);
		
		void SetupViewer();
		
		void AddViewerToWidget();
		void AddObject
		(
			std::string path,
			std::string fileName,
			std::string ext
		);

		void ReplaceWithObject(
			std::string path,
			std::string fileName,
			std::string ext
		);

		static const std::string pcd; 
		static const std::string ply;
		static const std::string stl;
		static const std::string proj;

		private slots:

			void on_actionImport_triggered();
			void on_actionReplace_triggered();
			void on_treeWidget_itemActivated(QTreeWidgetItem *item, int column);
};

#endif
