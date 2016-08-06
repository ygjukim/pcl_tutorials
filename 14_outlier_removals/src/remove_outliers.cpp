#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl;

bool conditional_removal(false), radius_based_removal(false), statistical_removal(false);


void printUsage(const char* progName)
{
	cout << "\nUsage: " << progName << " [options] pcd_file\n"
		 << "Options:\n"
		 << "----------------------------------------------------------\n"
		 << " -c\tapply conditional removal filter\n"
		 << " -r\tapply radius-based outlier removal filter\n"
		 << " -r\tapply statistical outlier removal filter\n"
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
	if (console::find_argument(argc, argv, "-c") >= 0)
	{
		conditional_removal = true;
	}
	else if (console::find_argument(argc, argv, "-r") >= 0)
	{
		radius_based_removal = true;
	}
	else if (console::find_argument(argc, argv, "-s") >= 0)
	{
		statistical_removal = true;
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
		cout << "Set a PCD file to be filtered...\n";
		printUsage(argv[0]);
		return -1;
	}

	// load source point cloud fron input file
	PointCloud<PointXYZ>::Ptr source_cloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr filtered_cloud(new PointCloud<PointXYZ>);

	cout << "load the PCD file '" << argv[indices[0]] << "'\n";
	if (io::loadPCDFile(argv[indices[0]], *source_cloud) < 0)
	{
		cout << "Error in loading the point cloud '" << argv[indices[0]] << "'...\n";
		return -1;
	}

	if (conditional_removal)
	{
/*
		PassThrough<PointXYZ> filter;
		filter.setInputCloud(source_cloud);
		filter.setFilterFieldName("z");
		filter.setFilterLimits(0.0, 2.0);
		filter.setFilterLimitsNegative(false);

		filter.filter(*filtered_cloud);
*/
		ConditionAnd<PointXYZ>::Ptr condition(new ConditionAnd<PointXYZ>);
		condition->addComparison(FieldComparison<PointXYZ>::ConstPtr 
			(new FieldComparison<PointXYZ>("z", ComparisonOps::GT, 0.0)));
		condition->addComparison(FieldComparison<PointXYZ>::ConstPtr 
			(new FieldComparison<PointXYZ>("z", ComparisonOps::LT, 2.0)));

		ConditionalRemoval<PointXYZ> cr_filter;
		cr_filter.setInputCloud(source_cloud);
		cr_filter.setCondition(condition);
		cr_filter.setKeepOrganized(true);
		cr_filter.setUserFilterValue(0.0);

		cr_filter.filter(*filtered_cloud);
	}
	else if (radius_based_removal) 
	{
		RadiusOutlierRemoval<PointXYZ> ror_filter;
		ror_filter.setInputCloud(source_cloud);
		ror_filter.setRadiusSearch(0.15);
		ror_filter.setMinNeighborsInRadius(10);

		ror_filter.filter(*filtered_cloud);
	}
	else if (statistical_removal)
	{
		StatisticalOutlierRemoval<PointXYZ> sor_filter;
		sor_filter.setInputCloud(source_cloud);
		sor_filter.setMeanK(50);
		sor_filter.setStddevMulThresh(1.0);

		sor_filter.filter(*filtered_cloud);
	}

	pcl::visualization::CloudViewer viewer("Filter Viewer");
	viewer.showCloud(filtered_cloud);

	while (!viewer.wasStopped()) {}

	return 0;
}