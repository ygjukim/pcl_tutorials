#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl;

string filter_field = "z";
float limit_value = 3.0;
bool is_negative = false;

void printUsage(const char* progName)
{
	cout << "\nUsage: " << progName << " [options] pcd_file\n"
		 << "Options:\n"
		 << "----------------------------------------------------------\n"
		 << " -f filed_name\tset a filtering field(defualt: " << filter_field << ")\n"
		 << " -l limit_value(float)\tset a filtering limit value(default: " << limit_value << ")\n"
		 << " -n \tset filter negatibe mode to true(default: " << is_negative << ")\n"
		 << " -h\tshow this help.\n";
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
	indices = console::parse_file_extension_argument(argc, argv, ".pcd");
	if (!indices.size()) 
	{
		cout << "Set a PCD file to be filtered...\n";
		printUsage(argv[0]);
		return -1;
	}	
	console::parse(argc, argv, "-f", filter_field);
	console::parse(argc, argv, "-l", limit_value);
	if (console::find_argument(argc, argv, "-n") >= 0)
	{
		is_negative = true;
	}

	cout << "PCD file: " << argv[indices[0]] << endl;
	cout << "Filte field: " << filter_field << endl;
	cout << "Limit value: " << limit_value << endl; 
	cout << "Negative mode: " << is_negative << endl; 

	// load source point cloud fron input file
	PointCloud<PointXYZ>::Ptr source_cloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr filtered_cloud(new PointCloud<PointXYZ>);

	if (io::loadPCDFile(argv[indices[0]], *source_cloud) < 0)
	{
		cout << "Error in loading the point cloud '" << argv[indices[0]] << "'...\n";
		return -1;
	}

	PassThrough<PointXYZ> filter;
	filter.setInputCloud(source_cloud);
	filter.setFilterFieldName(filter_field);
	filter.setFilterLimits(0.0, limit_value);
	filter.setFilterLimitsNegative(is_negative);

	filter.filter(*filtered_cloud);

	pcl::visualization::CloudViewer viewer("Filter Viewer");
	viewer.showCloud(filtered_cloud);

	while (!viewer.wasStopped()) {}

	return 0;
}