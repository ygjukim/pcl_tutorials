#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl;

void printUsage(char* progName)
{
	cout <<"\nUsage: " << progName << " [options] cloud_file.[pcd|ply]\n"
		 <<"Options: -h\tshow this help.\n";
}

int main(int argc, char** argv)
{
	// parse command line arguments
	if (argc < 2 || console::find_argument(argc, argv, "-h") >= 0 ||  console::find_argument(argc, argv, "--help") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}

	vector<int> indices;
	bool file_is_pcd(false);

	cout << "file name : " << argv[1] << endl;

	indices = console::parse_file_extension_argument(argc, argv, ".ply");
	if (!indices.size()) 
	{
		indices = console::parse_file_extension_argument(argc, argv, ".pcd");
		if (!indices.size())
		{
			printUsage(argv[0]);
			return -1;
		}
		else
		{
			file_is_pcd = true;
		}
	}

	// load source point cloud fron input file
	PointCloud<PointXYZ>::Ptr source_cloud(new PointCloud<PointXYZ>);

	if (file_is_pcd)
	{
		if (io::loadPCDFile(argv[indices[0]], *source_cloud) < 0)
		{
			cout << "Error in loading the point cloud '" << argv[indices[0]] << "'...\n";
			return -1;
		}
	}
	else 
	{
		if (io::loadPLYFile(argv[indices[0]], *source_cloud) < 0)
		{
			cout << "Error in loading the point cloud '" << argv[indices[0]] << "'...\n";
			return -1;
		}
	}

	// define rotation and translation matrix
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

	float theta = M_PI / 4;	// the angle of rotation in radian
	transform_1(0, 0) = cos(theta);
	transform_1(0, 1) = -sin(theta);
	transform_1(1, 0) = sin(theta);
	transform_1(1, 1) = cos(theta);

	transform_1(0, 3) = 2.5;	// a translation of 2.5 meters on the x axis

	cout << "Method #1: using a Matrix4f\n"
		 << transform_1 << endl;

	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	
	transform_2.translation() << 2.5, 0.0, 0.0;
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

	cout << "Method #2: using an Affine3f\n"
		 << transform_2.matrix() << endl;

	PointCloud<PointXYZ>::Ptr transform_cloud(new PointCloud<PointXYZ>);
	transformPointCloud(*source_cloud, *transform_cloud, transform_2);

	cout << "\nPoint cloud colors:\n"
		 << "  white - original point cloud\n"
		 << "  red - transformed point cloud\n";
	
	visualization::PCLVisualizer viewer("Matrix Transformation Viewer");
	visualization::PointCloudColorHandlerCustom<PointXYZ> source_cloud_color_handler(source_cloud, 255,255,255);
	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "origial_cloud");
	visualization::PointCloudColorHandlerCustom<PointXYZ> transform_cloud_color_handler(transform_cloud, 255, 0, 0);
	viewer.addPointCloud(transform_cloud, transform_cloud_color_handler, "transformed_cloud");
	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "origial_cloud");
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}

	return 0;
}