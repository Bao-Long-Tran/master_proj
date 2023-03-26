//#define _CRT_SECURE_NO_WARNINGS
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/sample_consensus/sac_model_sphere.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/filters/conditional_removal.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//
//#include <pcl/features/don.h>
//
//using namespace pcl;
//
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//
//int main() {
//	PointCloudT::Ptr inputCloud(new PointCloudT);
//	PointCloudT::Ptr cloud_filtered(new PointCloudT);
//	PointCloudT::Ptr cloud_f(new PointCloudT);
//	PointCloudT::Ptr final(new PointCloudT);
//	PointCloudT::Ptr cloud_cluster(new PointCloudT);
//
//
//	/*.........Load input cloud..........*/
//	pcl::PCDReader reader;
//	reader.read("pointCloud_Img29.pcd", *cloud_filtered);
//
//	// Create the filtering object: downsample the dataset using a leaf size of 1cm
//	pcl::VoxelGrid<pcl::PointXYZ> vg;
//	vg.setInputCloud(cloud_filtered);
//	vg.setLeafSize(5.0f, 5.0f, 5.0f);
//	vg.filter(*cloud_filtered);
//
//	pcl::io::savePCDFile("cloud_filtered_Z.pcd", *cloud_filtered, true);
//	std::cout << "save done" << std::endl;
//
//	/*........plannar segmentation.........*/
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PCDWriter writer;
//	seg.setOptimizeCoefficients(true);
//	seg.setModelType(pcl::SACMODEL_PLANE);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setMaxIterations(100);
//	seg.setDistanceThreshold(10);
//
//	int i = 0, nr_points = (int)cloud_filtered->size();
//
//	while (cloud_filtered->size() > 0.3 * nr_points)
//	{
//		// Segment the largest planar component from the remaining cloud
//		seg.setInputCloud(cloud_filtered);
//		seg.segment(*inliers, *coefficients);
//		if (inliers->indices.size() == 0)
//		{
//			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//			break;
//		}
//
//		cout << "inliers size: " << inliers->indices.size() << endl;
//
//		// Extract the planar inliers from the input cloud
//		pcl::ExtractIndices<pcl::PointXYZ> extract;
//		extract.setInputCloud(cloud_filtered);
//		extract.setIndices(inliers);
//		extract.setNegative(false);
//
//		// Get the points associated with the planar surface
//		extract.filter(*cloud_plane);
//
//		// Remove the planar inliers, extract the rest
//		extract.setNegative(true);
//		extract.filter(*cloud_f);
//		*cloud_filtered = *cloud_f;
//
//		//std::cout << "PointCloud representing the planar component: " << cloud_filtered->size() << " data points." << std::endl;
//	}
//
//	// Create the filtering object
//	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//	sor.setInputCloud(cloud_filtered);
//	sor.setMeanK(40);
//	sor.setStddevMulThresh(0.7);
//	sor.filter(*cloud_filtered);
//
//	pcl::io::savePCDFile("cloud_filter_Img29.pcd", *cloud_f, true);
//	std::cout << "save done" << std::endl;
//
//	///*......viewer........*/
//	pcl::visualization::PCLVisualizer viewer;
//	viewer.addPointCloud(cloud_filtered, "hehe");
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce();
//	}
//
//	//system("PAUSE");
//}