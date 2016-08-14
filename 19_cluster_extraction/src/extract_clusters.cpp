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

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		cout << "Input a PCD file name...\n";
		return 0;
	}

	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>), cloud_f(new PointCloud<PointXYZ>);
	PCDReader reader;
	reader.read(argv[1], *cloud);
	cout << "PointCloud before filtering has: " << cloud->points.size() << " data points.\n";

	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
	VoxelGrid<PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points.\n";

	SACSegmentation<PointXYZ> seg;
	PointIndices::Ptr inliers(new PointIndices);
	PointCloud<PointXYZ>::Ptr cloud_plane(new PointCloud<PointXYZ>);

	ModelCoefficients::Ptr coefficients(new ModelCoefficients);
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	int i=0, nr_points = (int)cloud_filtered->points.size();
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			cout << "Coud not estimate a planar model for the given dataset.\n";
			break;
		}

		ExtractIndices<PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_plane);
		cout << "PointCloud representing the planar component has: " << cloud_filtered->points.size() << " data points.\n";

		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered->swap(*cloud_f);
	}

	search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
	kdtree->setInputCloud(cloud_filtered);

	vector<PointIndices> cluster_indices;
	EuclideanClusterExtraction<PointXYZ> ece;
	ece.setClusterTolerance(0.02);
	ece.setMinClusterSize(100);
	ece.setMaxClusterSize(25000);
	ece.setSearchMethod(kdtree);
	ece.setInputCloud(cloud_filtered);
	ece.extract(cluster_indices);

	PCDWriter writer;
	int j = 0;
	for (vector<PointIndices>::const_iterator it=cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		PointCloud<PointXYZ>::Ptr cluster_cloud(new PointCloud<PointXYZ>);
		for (vector<int>::const_iterator pit=it->indices.begin(); pit != it->indices.end(); ++pit)
			cluster_cloud->points.push_back(cloud_filtered->points[*pit]);
		cluster_cloud->width = cluster_cloud->points.size();
		cluster_cloud->height = 1;
		cluster_cloud->is_dense = true;

		cout << "PointCloud representing a cluster has: " << cluster_cloud->points.size() << " data points.\n";

		stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<PointXYZ>(ss.str(), *cluster_cloud, false);
		j++;
	}

	return 0;
}