#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
	PCLPointCloud2::Ptr cloud_blob(new PCLPointCloud2), cloud_filtered_blob(new PCLPointCloud2);
	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>),
		cloud_p(new PointCloud<PointXYZ>), cloud_f(new PointCloud<PointXYZ>);

	PCDReader reader;
	reader.read("../table_scene_lms400.pcd", *cloud_blob);
	cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points\n";

	VoxelGrid<PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered_blob);

	fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);
	cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points\n";

	PCDWriter writer;
	writer.write<PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

	ModelCoefficients::Ptr coefficients(new ModelCoefficients());
	PointIndices::Ptr inliers(new PointIndices());
	SACSegmentation<PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);

	ExtractIndices<PointXYZ> extract;

	int i = 0, nr_points = (int)cloud_filtered->points.size();
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			cerr << "Could not estimate a planar model for the given dataset.\n";
			break;
		}

		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		cerr << "PointCloud representing the plane component: " << cloud_p->width * cloud_p->height << " data points.\n";

		stringstream ss;
		ss << "table_scene_lms400_plane_" << i << ".pcd";
		writer.write<PointXYZ>(ss.str(), *cloud_p, false);

		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);
		i++;
	}

	return 0;
}
