//#define _CRT_SECURE_NO_WARNINGS 
////#include <stdio.h>
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//#include <iostream>
//#include <pcl/common/io.h>
//#include <pcl/point_types.h>								// Lib for initialization point XYZ
//#include <pcl/registration/icp.h>
//#include <pcl/visualization/pcl_visualizer.h>				// Lib for visualization
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/histogram_visualizer.h>
//#include <pcl/features/pfh.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/integral_image_normal.h>
//#include <pcl/visualization/histogram_visualizer.h>
//#include <pcl/visualization/pcl_plotter.h>
//#include <pcl/filters/voxel_grid.h>
//
//int main(int argc, char** argv) {
//	// To estimate Point Feature Histogram we need point cloud and normals
//	// First we need to estimate normals of point cloud
//
//	// Object for storing input cloud
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//	// Object for storing normals
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal> ());
//	// Object for storing PFH descriptors for each point
//	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125> ());
//
//	// Read a PCD file into input cloud
//	pcl::io::loadPCDFile("table_scene_mug_stereo_textured.pcd", *cloud);
//
//	// Create the filtering object: downsample the dataset using a leaf size of 1cm
//	pcl::VoxelGrid<pcl::PointXYZ> vg;
//	vg.setInputCloud(cloud);
//	vg.setLeafSize(0.003, 0.003, 0.003);
//	vg.filter(*cloud_filtered);
//
//	std::cout << "filtered cloud size: " << cloud_filtered->size() << std::endl;
//	std::cout << "Start estimate normals" << std::endl;
//	// SURFACE NORMALS ESTIMATE
//	// Estimate normals of input point cloud
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//	ne.setInputCloud(cloud_filtered);
//	// Create a Kdtree for searching method
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
//	ne.setSearchMethod(kdtree);
//	ne.setRadiusSearch(0.006);
//	ne.compute(*normals);
//
//	////// NORMALS ESTIMATE BY INTEGRAL 
//	////pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//	////ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
//	////ne.setMaxDepthChangeFactor(0.02f);
//	////ne.setNormalSmoothingSize(10.0f);
//	////ne.setInputCloud(cloud);
//	////ne.compute(*normals);
//
//	std::cout << "Start calculate PFH" << std::endl;
//	// After having point cloud and normals, we estimate PFH
//	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
//	pfh.setInputCloud(cloud_filtered);
//	pfh.setInputNormals(normals);
//
//	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//	pfh.setSearchMethod(kdtree);
//	// This searching method looks for neighbors
//	pfh.setRadiusSearch(0.01);
//	pfh.compute(*descriptors);
//
//	std::cout << "descriptor size " << descriptors->points.size() << std::endl;
//
//	pcl::visualization::PCLPlotter PFHviewer;
//	PFHviewer.addFeatureHistogram<pcl::PFHSignature125>(*descriptors, 73712);
//	PFHviewer.plot();
//
//	//// Visualize
//	//pcl::visualization::PCLVisualizer viewer;
//	//viewer.setBackgroundColor(0, 0, 0.5);
//	//viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_filtered, normals);
//	////viewer.addPointCloud(cloud_filtered);
//	//while (!viewer.wasStopped()) {
//	//	viewer.spinOnce();
//	//}
//
//	system("PAUSE");
//}