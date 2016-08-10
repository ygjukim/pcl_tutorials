#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl;

void printUsage(const char *progName)
{
	cout << "Usage: " << progName << " [-d|-k|-u] pcd_file\n"
		 << "Options:\n"
		 << "  -d\tdownsampling using VoxelGrid filter\n"
		 << "  -k\tchoosing keypoints using Uniform Sampling\n"
		 << "  -u\tupsampling usin MLS(Moving Least Squares)\n"
		 << "  -h\tshow this help\n"
		 << "\n";
}

int main(int argc, char** argv)
{
	//
	// Parse command line arguments
	// 
	if (console::find_argument(argc, argv, "-h") >=0)
	{
		printUsage(argv[0]);
		return 0;
	}

	bool downsampling(false), uniform_sampling(false), upsampling(false), resampled(false);

	if (console::find_argument(argc, argv, "-d") >=0)
	{
		downsampling = true;
	}
	else if (console::find_argument(argc, argv, "-k") >=0)
	{
		uniform_sampling = true;
	}
	else if (console::find_argument(argc, argv, "-u") >=0)
	{
		upsampling = true;
	}
	else 
	{
		printUsage(argv[0]);
		return 0;
	}

	vector<int> indices;
	indices = console::parse_file_extension_argument(argc, argv, ".pcd");
	if (!indices.size()) 
	{
		cout << "Set a PCD file to be resampled...\n";
		printUsage(argv[0]);
		return -1;
	}

	//
	// load input PCD file to PointCloud
	// 
	PointCloud<PointXYZ>::Ptr source_cloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr filtered_cloud(new PointCloud<PointXYZ>);

	if (io::loadPCDFile(argv[indices[0]], *source_cloud) != 0)
	{
		cout << "Error in loading the point cloud '" << argv[indices[0]] << "'...\n";
		return -1;
	}

	//
	// resample source PointCloud
	//
	
	if (downsampling)
	{
		VoxelGrid<PointXYZ> filter;
		filter.setInputCloud(source_cloud);
		// We set the size of every voxel to be 1x1x1cm
		// (only one point per every cubic centimeter will survive).
		filter.setLeafSize(0.01f, 0.01f, 0.01f);

		filter.filter(*filtered_cloud);
	}
	else if (uniform_sampling)
	{
		// Uniform sampling object.
		UniformSampling<PointXYZ> filter;
		filter.setInputCloud(source_cloud);
		filter.setRadiusSearch(0.01f);
		// We need an additional object to store the indices of surviving points.

		PointCloud<int> keypointIndices;
		filter.compute(keypointIndices);
		copyPointCloud(*source_cloud, keypointIndices.points, *filtered_cloud);
	}
	else if (upsampling)
	{
		// Filtering object.
		MovingLeastSquares<PointXYZ, PointXYZ> filter;
		filter.setInputCloud(source_cloud);
		// Object for searching.
		search::KdTree<PointXYZ>::Ptr kdtree;
		filter.setSearchMethod(kdtree);
		// Use all neighbors in a radius of 3cm.
		filter.setSearchRadius(0.03);
		// Upsampling method. Other possibilites are DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
		// and VOXEL_GRID_DILATION. NONE disables upsampling. Check the API for details.
		filter.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
		// Radius around each point, where the local plane will be sampled.
		filter.setUpsamplingRadius(0.03);
		// Sampling step size. Bigger values will yield less (if any) new points.
		filter.setUpsamplingStepSize(0.02);

		filter.process(*filtered_cloud);
	}

	//
	// Show resampled PointCloud via CloudViewer
	//
	visualization::CloudViewer viewer("Resampling Viewer");
	viewer.showCloud(filtered_cloud);

	while (!viewer.wasStopped()) {}

	return 0;
}