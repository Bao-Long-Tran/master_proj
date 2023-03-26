//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//
//#include <opencv2\opencv.hpp>
//
//#include <iostream>
//#include <sstream>
//
//using namespace cv;
//using namespace std;
//using namespace pcl;
//
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//
//int main() {
//
//	PointCloudT::Ptr depth_cloud(new PointCloudT); //Recovered point cloud
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("pointcloud"));
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZ>);
//
//	pcl::PCDReader reader;
//
//
//	reader.read("table_scene_lms400.pcd", *cloud_in);
//
//
//	viewer->addPointCloud<PointT>(cloud_in, "depth_cloud");
//
//	cout << "Size" + to_string(cloud_in->size()) << endl;
//
//	/*........Convert PCD -> Image...........*/
//	//Mat segmentedImg = imread("colorImage9.png", IMREAD_COLOR);
//
//	//Mat color_Img = imread("colorImage9.png", IMREAD_COLOR);
//
//	//int width = segmentedImg.cols;
//	//int height = segmentedImg.rows;
//
//	//for (int y = 0; y < height; y++) {
//	//	for (int x = 0; x < width; x++) {			
//	//		segmentedImg.at<Vec3b>(y, x)[2] = 0;
//	//		segmentedImg.at<Vec3b>(y, x)[1] = 0;
//	//		segmentedImg.at<Vec3b>(y, x)[0] = 0;
//	//	}
//	//}
//
//	//cout << "height: " << height << endl;
//	//cout << "width: " << width << endl;
//
//	//for (int k = 0; k < cloud_in->size(); k++) {
//
//	//				//float x_val = (x - 320)*0.00173667*depth;
//	//				//float y_val = (y - 240)*0.00173667*depth;
//
//	//	float y_val = cloud_in->points[k].y; float x_val = cloud_in->points[k].x; float depth = cloud_in->points[k].z;
//
//	//	//cout << to_string(depth) << endl;
//
//	//	int y = (y_val / (depth * 0.00173667) + 240);
//	//	int x = (x_val / (depth * 0.00173667) + 320);
//
//	//	if (x > 0 && x < width && y > 0 && y < height) {
//	//		//segmentedImg.at<Vec3b>(y, x)[2] = color_Img.at<Vec3b>(y, x)[2];
//	//		//segmentedImg.at<Vec3b>(y, x)[1] = color_Img.at<Vec3b>(y, x)[1];
//	//		//segmentedImg.at<Vec3b>(y, x)[0] = color_Img.at<Vec3b>(y, x)[0];
//
//	//		segmentedImg.at<Vec3b>(y, x)[2] = 255;
//	//		segmentedImg.at<Vec3b>(y, x)[1] = 255;
//	//		segmentedImg.at<Vec3b>(y, x)[0] = 255;
//	//	}
//	//}
//
//	//imshow("Display window", segmentedImg);
//	//imwrite("improved_cy_Img9.png", segmentedImg);
//	//waitKey(0);
//
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce();
//	}
//
//	system("PAUSE");
//}