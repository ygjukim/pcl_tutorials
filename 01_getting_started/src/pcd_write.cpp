#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
	PointCloud<PointXYZ> cloud;

	cloud.width = 5;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (size_t i=0; i<cloud.points.size(); ++i)
	{
		cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	io::savePCDFileASCII("test_pcd.pcd", cloud);
	cerr << "Saved " << cloud.points.size() << " data points totest_pcd.pcd." << endl;

	for (size_t i=0; i<cloud.points.size(); ++i)
	{
		cerr << "\t" << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << "\n";
	}

	return 0;
}