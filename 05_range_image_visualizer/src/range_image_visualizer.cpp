#include <iostream>

#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl;

typedef PointXYZ PointType;

float angular_resolution_x = 0.5f,
	  angular_resolution_y = angular_resolution_x;
RangeImage::CoordinateFrame coordinate_frame = RangeImage::CAMERA_FRAME;
bool live_update = false;

void printUsage(const char* progName)	  
{
	cout << "\nUsage: " << progName << " [options] <scene.pcd>\n"
		 << "Options:\n"
		 << "--------------------------------------------------------\n"
		 << "-rx <float> 	angular resolution in degree (default " << angular_resolution_x << ")\n"
		 << "-ry <float> 	angular resolution in degree (default " << angular_resolution_y << ")\n"
		 << "-c <int>		coordinate frame (default " << (int)coordinate_frame << ")\n"
		 << "-l 			live update - update the range image according to the selected view in the 3D viewer.\n"
		 << "-h 			this help\n\n";
}

void setViewerPose(visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
							 look_at_vector[0], look_at_vector[1], look_at_vector[2],
							 up_vector[0], up_vector[1], up_vector[2]);
}

int main(int argc, char** argv)
{
	if (console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}
	if (console::find_argument(argc, argv, "-l") >= 0)
	{
		live_update = true;
		cout << "Live update is on.\n";
	}
	if (console::parse(argc, argv, "-rx", angular_resolution_x) >= 0)
	{
		cout << "Setting angular resolution in x-direction to " << angular_resolution_x << "deg.\n";
	}
	if (console::parse(argc, argv, "-ry", angular_resolution_y) >= 0)
	{
		cout << "Setting angular resolution in y-direction to " << angular_resolution_y << "deg.\n";
	}
	int tmp_coordinate_frame;
	if (console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
	{
		coordinate_frame = RangeImage::CoordinateFrame(tmp_coordinate_frame);
		cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
	}
	angular_resolution_x = deg2rad(angular_resolution_x);
	angular_resolution_y = deg2rad(angular_resolution_y);

	PointCloud<PointType>::Ptr point_cloud_ptr(new PointCloud<PointType>);
	PointCloud<PointType>& point_cloud = *point_cloud_ptr;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
	vector<int> pcd_filename_indices = console::parse_file_extension_argument(argc, argv, "pcd");
	if (!pcd_filename_indices.empty()) 
	{
		string filename = argv[pcd_filename_indices[0]];
		if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
		{
			cout << "Was not able to open file " << filename << ".\n";
			printUsage(argv[0]);
			return 0;
		}
		scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0], 
									point_cloud.sensor_origin_[1], point_cloud.sensor_origin_[2]))
						  * Eigen::Affine3f(point_cloud.sensor_orientation_);
	}
	else 
	{
		cout << "\nNo *.pcd file given => Generating example point cloud..\n";
		for (float x = -0.5f; x <= 0.5f; x += 0.01f)
		{
			for (float y = -0.5f; y <= 0.5f; y += 0.01f)
			{
				PointType point;
				point.x = x;
				point.y = y;
				point.z = 2.0f - y;
				point_cloud.points.push_back(point);
			}
		}
		point_cloud.width = (int)point_cloud.points.size(); point_cloud.height = 1;
	}

	// Create RangeImage from the point cloud
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<RangeImage> range_image_ptr(new RangeImage);
	RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
									 deg2rad(360.0f), deg2rad(180.0f), scene_sensor_pose,
									 coordinate_frame, noise_level, min_range, border_size);

	visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	visualization::PointCloudColorHandlerCustom<PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	//viewer.addCoordinateSystem (1.0f, "global");
  	//PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  	//viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  	viewer.initCameraParameters();
  	setViewerPose(viewer, range_image.getTransformationToWorldSystem());

  	visualization::RangeImageVisualizer range_image_widget("Range Image");
  	range_image_widget.showRangeImage(range_image);

  	while (!viewer.wasStopped())
  	{
  		range_image_widget.spinOnce();
  		viewer.spinOnce();
  		pcl_sleep(0.01);

  		if (live_update)
  		{
  			scene_sensor_pose = viewer.getViewerPose();
			range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
											 deg2rad(360.0f), deg2rad(180.0f), scene_sensor_pose,
											 RangeImage::LASER_FRAME, noise_level, min_range, border_size);
			range_image_widget.showRangeImage(range_image);
  		}
  	}
}