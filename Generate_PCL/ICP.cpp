//#define _CRT_SECURE_NO_WARNINGS 
////#include <stdio.h>
//#include <iostream>
//#include <pcl/common/io.h>
//#include <pcl/point_types.h>								// Lib for initialization point XYZ
//#include <pcl/registration/icp.h>
//#include <pcl/visualization/pcl_visualizer.h>				// Lib for visualization
//#include <pcl/console/time.h>								// TicToc
//#include <pcl/io/pcd_io.h>
//#include <pcl/common/transforms.h>
//#include <pcl/filters/voxel_grid.h>
////#include <pcl/visualization/cloud_viewer.h>
//
//using namespace pcl;
//
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//
//bool next_iteration = false;
//
//
//int main(int argc, char* argv[])
//{
//	// The point cloud will be using
//	PointCloudT::Ptr cloud_in(new PointCloudT);
//	PointCloudT::Ptr cloud_tr(new PointCloudT);
//	PointCloudT::Ptr cloud_icp(new PointCloudT);
//
//	PointCloudT::Ptr cloud_fittered_in(new PointCloudT);
//	PointCloudT::Ptr cloud_fittered_icp(new PointCloudT);
//
//	int iterations = 1;  // Default number of ICP iterations
//
//	pcl::console::TicToc time;
//	time.tic();
//
//	pcl::io::loadPCDFile("PCL_box_img1_2_3_4_5_old.pcd", *cloud_in);
//	pcl::io::loadPCDFile("PCL_box_img2_3_4_5_6_old.pcd", *cloud_icp);
//
//
//	// Downsample the point cloud
//	// Create the filtering object: downsample the dataset using a leaf size of 1cm
//	pcl::VoxelGrid<pcl::PointXYZ> vgIN;
//	vgIN.setInputCloud(cloud_in);
//	vgIN.setLeafSize(5.0f, 5.0f, 5.0f);
//	vgIN.filter(*cloud_in);
//
//	pcl::VoxelGrid<pcl::PointXYZ> vgICP;
//	vgICP.setInputCloud(cloud_icp);
//	vgICP.setLeafSize(5.0f, 5.0f, 5.0f);
//	vgICP.filter(*cloud_icp);
//
//
//	pcl::IterativeClosestPoint<PointT, PointT> icp;
//	icp.setMaximumIterations(200);
//	icp.setInputSource(cloud_icp);
//	icp.setInputTarget(cloud_in);
//	icp.align(*cloud_icp);
//	icp.setMaximumIterations(1);
//
//	if (icp.hasConverged()) {
//		std::cout << "Good" << endl;
//	}
//	else {
//		std::cout << "Fail" << endl;
//	}
//
//	*cloud_icp+= *cloud_in;
//
//	pcl::io::savePCDFile("PCL_box_img1_2_3_4_5_6_old.pcd", *cloud_icp, true);
//
//	pcl::visualization::PCLVisualizer viewer("hello");
//	viewer.addPointCloud(cloud_in, "original");
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud_icp, 255, 0, 0);
//	viewer.addPointCloud(cloud_icp, colorHandler, "tranformed");
//
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce();
//	}
//}