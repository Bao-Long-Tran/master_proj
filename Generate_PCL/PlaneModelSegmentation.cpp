//#define _CRT_SECURE_NO_WARNINGS
//
////*.............Euclidean Cluster Extraction ..........* //
//#include <pcl/ModelCoefficients.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/visualization/cloud_viewer.h>
//
//int main() {
//	// Read in the cloud data
//	pcl::PCDReader reader;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
//
//	reader.read("PCL_img26_new.pcd", *cloud);
//
//	// Create the filtering object: downsample the dataset using a leaf size of 1cm
//	pcl::VoxelGrid<pcl::PointXYZ> vg;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//	vg.setInputCloud(cloud);
//	vg.setLeafSize(3.0f, 3.0f, 3.0f);
//	vg.filter(*cloud_filtered);
//
//	// Create the segmentation object for the planar model and set all the parameters
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PCDWriter writer;
//	seg.setOptimizeCoefficients(true);
//	seg.setModelType(pcl::SACMODEL_PLANE);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setMaxIterations(100);
//	seg.setDistanceThreshold(7);
//
//	int i = 0, nr_points = (int)cloud_filtered->size();
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
//		// Extract the planar inliers from the input cloud
//		pcl::ExtractIndices<pcl::PointXYZ> extract;
//		extract.setInputCloud(cloud_filtered);
//		extract.setIndices(inliers);
//		extract.setNegative(false);
//
//		// Get the points associated with the planar surface
//		extract.filter(*cloud_plane);
//		std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;
//
//		// Remove the planar inliers, extract the rest
//		extract.setNegative(true);
//		extract.filter(*cloud_f);
//		*cloud_filtered = *cloud_f;
//	}
//
//	writer.write<pcl::PointXYZ>("plane_segmented_Image34.pcd", *cloud_filtered, false);
//
//	pcl::visualization::PCLVisualizer viewer;
//	viewer.addPointCloud(cloud_filtered, "hehe");
//
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce();
//	}
//
//	std::cout << "Done" << std::endl;
//	system("PAUSE");
//}