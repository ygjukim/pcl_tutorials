#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace pcl;

boost::shared_ptr<visualization::PCLVisualizer>
simpleVis(PointCloud<PointXYZ>::ConstPtr cloud)
{
	boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0, "global");
	viewer->initCameraParameters();
	return viewer;
}

int main(int argc, char** argv)
{
	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr final (new PointCloud<PointXYZ>);

	cloud->width = 500;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i=0; i<cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.00);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.00);
		if (console::find_argument(argc, argv, "-s") >= 0 || console::find_argument(argc, argv, "-sf") >= 0)
		{
			if (i % 5 == 0)
				cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.00);
			else if (i % 2 == 0)
				cloud->points[i].z = sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
										   - (cloud->points[i].y * cloud->points[i].y));
			else
				cloud->points[i].z = -sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
										   - (cloud->points[i].y * cloud->points[i].y));
		}
		else
		{
			if (i % 2 == 0)
				cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.00);
			else
				cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
		}
	}

	vector<int> inliers;

	SampleConsensusModelSphere<PointXYZ>::Ptr 
		model_s(new SampleConsensusModelSphere<PointXYZ>(cloud));
	SampleConsensusModelPlane<PointXYZ>::Ptr
		model_p(new SampleConsensusModelPlane<PointXYZ>(cloud));
	if (console::find_argument(argc, argv, "-f") >= 0)
	{
		RandomSampleConsensus<PointXYZ> ransac (model_p);
		ransac.setDistanceThreshold(0.01);
		ransac.computeModel();
		ransac.getInliers(inliers);
	}
	else if (console::find_argument(argc, argv, "-sf") >= 0)
	{
		RandomSampleConsensus<PointXYZ> ransac (model_s);
		ransac.setDistanceThreshold(0.01);
		ransac.computeModel();
		ransac.getInliers(inliers);
	}

	copyPointCloud<PointXYZ>(*cloud, inliers, *final);

	boost::shared_ptr<visualization::PCLVisualizer> viewer;
	if (console::find_argument(argc, argv, "-f") >= 0 || console::find_argument(argc, argv, "-sf") >= 0)
		viewer = simpleVis(final);
	else
		viewer = simpleVis(cloud);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}