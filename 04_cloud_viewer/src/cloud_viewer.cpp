#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ pt;
	pt.x = 1.0;
	pt.y = 0;
	pt.z = 0;
	viewer.addSphere(pt, 0.25, "sphere", 0);
	std::cout << "I only run once..." << std::endl;
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++ << std::endl;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 200, "text", 0);

	user_data++;
}

void printUsage(const char* programName)
{
	std::cout << "Usage: " << programName << " pcd-file" << std::endl;
}

int main(int argc, char** argv)
{
	if (argc < 2) {
		printUsage("cloud_viewer");
		exit(-1);
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile(argv[1], *cloud);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	viewer.showCloud(cloud);

	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	viewer.runOnVisualizationThread(viewerPsycho);

	while (!viewer.wasStopped()) 
	{
		user_data++;
	}

	return 0;
}