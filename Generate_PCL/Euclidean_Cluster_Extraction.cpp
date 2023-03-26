#define _CRT_SECURE_NO_WARNINGS

//*.............Euclidean Cluster Extraction ..........* //
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
#include <pcl/visualization/cloud_viewer.h>

int main()
{
	// Read in the cloud data
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("table_scene_lms400.pcd", *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl; //*

																									   // Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl; //*

																											   // Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	int nr_points = (int)cloud_filtered->size();
	while (cloud_filtered->size() > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud_filtered = *cloud_f;
	}

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			pcl::PointXYZRGB point;
			point.x = cloud_filtered->points[*pit].x;
			point.y = cloud_filtered->points[*pit].y;
			point.z = cloud_filtered->points[*pit].z;

			if (j == 0) //Red	#FF0000	(255,0,0)
			{
				point.r = 0;
				point.g = 0;
				point.b = 255;
			}
			else if (j == 1) //Lime	#00FF00	(0,255,0)
			{
				point.r = 0;
				point.g = 255;
				point.b = 0;
			}
			else if (j == 2) // Blue	#0000FF	(0,0,255)
			{
				point.r = 255;
				point.g = 0;
				point.b = 0;
			}
			else if (j == 3) // Yellow	#FFFF00	(255,255,0)
			{
				point.r = 255;
				point.g = 255;
				point.b = 0;
			}
			point_cloud_segmented->push_back(point);
		}

		j++;
	}

	//	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

	//	//for (const auto& idx : it->indices)
	//	//	cloud_cluster->push_back((*cloud_filtered)[idx]); //*
	//	//cloud_cluster->width = cloud_cluster->size();
	//	//cloud_cluster->height = 1;
	//	//cloud_cluster->is_dense = true;

	//	//std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
	//	//std::stringstream ss;
	//	//ss << "cloud_cluster_" << j << ".pcd";
	//	//writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
	//	//j++;
	//}

	//writer.write<pcl::PointXYZ>("plane_segmented_Image34.pcd", *cloud_filtered, false);
	
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(point_cloud_segmented, "hehe");
	
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}