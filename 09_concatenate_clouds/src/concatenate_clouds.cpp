#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		cerr << "Please specify command line argument '-f' or '-p'\n";
		return 0;
	}

	PointCloud<PointXYZ> cloud_a, cloud_b, cloud_c;
	PointCloud<Normal> n_cloud_b;
	PointCloud<PointNormal> pn_cloud_c;

	cloud_a.width = 5;
	cloud_a.height = cloud_b.height = cloud_c.height = 1;
	cloud_a.points.resize(cloud_a.width * cloud_a.height);

	bool concatenate_points(false);
	if (strcmp(argv[1], "-p") == 0)
		concatenate_points = true;

	for (size_t i=0; i<cloud_a.points.size(); ++i)
	{
		cloud_a.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_a.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	if (concatenate_points)
	{
		cloud_b.width = 3;
		cloud_b.points.resize(cloud_b.width * cloud_b.height);
		for (size_t i=0; i<cloud_b.points.size(); ++i)
		{
			cloud_b.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud_b.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud_b.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		}		
	}
	else 
	{
		n_cloud_b.width = 5;
		n_cloud_b.height = 1;
		n_cloud_b.points.resize(n_cloud_b.width * n_cloud_b.height);
		for (size_t i=0; i<n_cloud_b.points.size(); ++i)
		{
			n_cloud_b.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
			n_cloud_b.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
			n_cloud_b.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);
		}		
	}

	cerr << "Cloud A: " << endl;
	for (size_t i=0; i<cloud_a.points.size(); ++i)
		cerr << "\t" << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << endl;

	cerr << "Cloud B: " << endl;
	if (concatenate_points)
	{
		for (size_t i=0; i<cloud_b.points.size(); ++i)
			cerr << "\t" << cloud_b.points[i].x << " " << cloud_b.points[i].y << " " << cloud_b.points[i].z << endl;
	}
	else 
	{
		for (size_t i=0; i<n_cloud_b.points.size(); ++i)
			cerr << "\t" << n_cloud_b.points[i].normal[0] << " " << n_cloud_b.points[i].normal[1] << " " << n_cloud_b.points[i].normal[2] << endl;
	}

	if (concatenate_points)
	{
		cloud_c = cloud_a;
		cloud_c += cloud_b;
		cerr << "Cloud C: " << endl;
		for (size_t i=0; i<cloud_c.points.size(); ++i)
			cerr << "\t" << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << endl;		
	}
	else
	{
		concatenateFields(cloud_a, n_cloud_b, pn_cloud_c);
		cerr << "Cloud C: " << endl;
		for (size_t i=0; i<pn_cloud_c.points.size(); ++i)
			cerr << "\t" << pn_cloud_c.points[i].x << " " << pn_cloud_c.points[i].y << " " << pn_cloud_c.points[i].z << " "
			      << pn_cloud_c.points[i].normal[0] << " " << pn_cloud_c.points[i].normal[1] << " " 
			      << pn_cloud_c.points[i].normal[2] << "\n";	
	}

	return 0;
}