//#define _CRT_SECURE_NO_WARNINGS
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/pcd_io.h>
//
//#include <opencv2\opencv.hpp>
//
//#include <iostream>
//#include <sstream>
//
//using namespace cv;
//using namespace std;
//using namespace std::chrono;
//
//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//
//int main() {
//	Mat colorMap = imread("colorImageTest_26.png", IMREAD_COLOR);
//	Mat depthMap = imread("depthMapImageTest_26.png", IMREAD_COLOR);
//
//	Mat objectMap = imread("Mask_Only_MRCNN.png", IMREAD_COLOR);
//
//	int width = depthMap.cols;
//	int height = depthMap.rows;
//
//	PointCloudT::Ptr depth_cloud(new PointCloudT); //Recovered point cloud
//
//	auto start = high_resolution_clock::now();
//
//	for (int y = 0; y < height; y++) {
//		for (int x = 0; x < width; x++) {
//			PointT newPoint;
//
//			unsigned char temp_r = depthMap.at<Vec3b>(y, x)[0];
//			unsigned char temp_g = depthMap.at<Vec3b>(y, x)[1];
//
//			float depth = ((temp_g + temp_r*256) * 8000 / 65536);
//
//			float x_val = (x - 320)*0.00173667*depth;
//			float y_val = (y - 240)*0.00173667*depth;
//
//			newPoint.x = x_val;
//			newPoint.y = y_val;
//			newPoint.z = depth;
//
//			if((objectMap.at<Vec3b>(y, x)[2] == 255) && (objectMap.at<Vec3b>(y, x)[1] == 0) && (objectMap.at<Vec3b>(y, x)[0] == 0))
//			{
//				newPoint.r = colorMap.at<Vec3b>(y, x)[2];
//				newPoint.g = colorMap.at<Vec3b>(y, x)[1];
//				newPoint.b = colorMap.at<Vec3b>(y, x)[0];
//			}
//			else {
//				newPoint.r = 0;
//				newPoint.g = 0;
//				newPoint.b = 0;
//			}
//
//			depth_cloud->points.push_back(newPoint);
//				
//		}
//	}
//
//	auto stop = high_resolution_clock::now();
//	auto duration = duration_cast<microseconds>(stop - start);
//
//	cout << "Time taken by function: " << duration.count() << " microseconds" << endl;
//
//	pcl::io::savePCDFile("PCL_img26_new.pcd", *depth_cloud, true);
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("pointcloud"));
//	viewer->addPointCloud<PointT>(depth_cloud, "depth_cloud");
//
//	cout << to_string(depth_cloud->size());
//
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce();
//	}
//
//	//system("PAUSE");
//}