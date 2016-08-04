#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl;

typedef PointXYZ 	PointType;

float angular_resolution = 0.5f;
RangeImage::CoordinateFrame coordinate_frame = RangeImage::CAMERA_FRAME;
//RangeImage::CoordinateFrame coordinate_frame = RangeImage::LASER_FRAME;
bool setUnseenToMaxRange = false;

void printUsage(const char* progName)
{
	cout << "\nUsage: " << progName << " [options] <scene.pcd>\n"
	 	 << "Options:\n"
	 	 << "-------------------------------------------------------\n"
	 	 << "\t-r <float>\tangular resolution in degrees (default " << angular_resolution << ") \n"
	 	 << "\t-c <int>\tcoordinate frame (default " << (int)coordinate_frame << ")\n"
	 	 << "\t-m\tTreat all unseen points to max range\n"
	 	 << "\t-h\tthis help\n\n";
}

int main(int argc, char** argv)
{
	//
	// Parse command line arguments
	// 
	if (console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}
	if (console::find_argument(argc, argv, "-m") >= 0)
	{
		setUnseenToMaxRange = true;
		cout << "Setting unseen values in range image to maximum range readings.\n";
	}
	int tmp_coordinate_frame;
	if (console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
	{
		coordinate_frame = RangeImage::CoordinateFrame(tmp_coordinate_frame);
		cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
	}
	if (console::parse(argc, argv, "-r", angular_resolution) >= 0)
	{
		cout << "Setting angular resolution tp " << angular_resolution << " deg.\n";
	}
	angular_resolution = deg2rad(angular_resolution);

	//
	// Read pcd file or create example point cloud if not given
	// 
	PointCloud<PointType>::Ptr point_cloud_ptr(new PointCloud<PointType>);
	PointCloud<PointType>& point_cloud = *point_cloud_ptr;
	PointCloud<PointWithViewpoint> far_ranges;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
	
	vector<int> pcd_files_indices = console::parse_file_extension_argument(argc, argv, "pcd");
	if (!pcd_files_indices.empty())
	{
		string filename = argv[pcd_files_indices[0]];
		if (io::loadPCDFile(filename, point_cloud) == -1)
		{
			cout << "Was not able to open file '" << filename << "'.\n";
			printUsage(argv[0]);
			return 0;
		}
		scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
								point_cloud.sensor_origin_[1], point_cloud.sensor_origin_[2]))
						  * Eigen::Affine3f(point_cloud.sensor_orientation_);

		string far_ranges_filename = getFilenameWithoutExtension(filename) + "_far_ranges.pcd";
		if (io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
		{
			cout << "Far ranges file '" << far_ranges_filename << "' does not exits.\n";
		}
	}
	else 
	{
	    cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
	    for (float x=-0.5f; x<=0.5f; x+=0.01f)
	    {
	      for (float y=-0.5f; y<=0.5f; y+=0.01f)
	      {
	        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
	        point_cloud.points.push_back (point);
	      }
	    }
	    point_cloud.width = (int) point_cloud.points.size ();  point_cloud.height = 1;
	}

	//
	// Create RangeImage from the point cloud
	// 
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<RangeImage> range_image_ptr (new RangeImage);
	RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud(point_cloud, angular_resolution, deg2rad(360.0f), deg2rad(180.0f),
									 scene_sensor_pose,coordinate_frame, noise_level, min_range, border_size);
	range_image.integrateFarRanges(far_ranges);
	if (setUnseenToMaxRange)
		range_image.setUnseenToMaxRange();

	//
	// Open 3D viewer and add point cloud
	// 
	visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(1.0f, "global");
	visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler(point_cloud_ptr, 0, 0, 0);
	viewer.addPointCloud(point_cloud_ptr, point_cloud_color_handler, "original point cloud");
	//PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 150, 150, 150);
	//viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
	//viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "range image");
	
	//
	// Extract borders 
	// 
	RangeImageBorderExtractor border_extractor(&range_image);
	PointCloud<BorderDescription> border_descriptions;
	border_extractor.compute(border_descriptions);

	//
	// Show points in 3D viewer
	// 
	PointCloud<PointWithRange>::Ptr border_points_ptr(new PointCloud<PointWithRange>),
								    veil_points_ptr(new PointCloud<PointWithRange>),
								    shadow_points_ptr(new PointCloud<PointWithRange>);
     PointCloud<PointWithRange> &border_points = *border_points_ptr;
     PointCloud<PointWithRange> &veil_points = *veil_points_ptr;
	PointCloud<PointWithRange> &shadow_points = *shadow_points_ptr;

   	for (int y=0; y<(int)range_image.height; ++y)
   	{
   		for (int x=0; x<(int)range_image.width; ++x)
   		{
   			int index = y * range_image.width + x;
   			if (border_descriptions.points[index].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
   				border_points.points.push_back(range_image.points[index]);
   			if (border_descriptions.points[index].traits[pcl::BORDER_TRAIT__VEIL_POINT])
   				veil_points.points.push_back(range_image.points[index]);
   			if (border_descriptions.points[index].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
   				shadow_points.points.push_back(range_image.points[index]);
   		}
   	}     						   								

   	visualization::PointCloudColorHandlerCustom<PointWithRange> border_points_color_handler(border_points_ptr, 0, 255, 0);
   	viewer.addPointCloud<PointWithRange>(border_points_ptr, border_points_color_handler, "border points");
   	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
   	visualization::PointCloudColorHandlerCustom<PointWithRange> veil_points_color_handler(veil_points_ptr, 255, 0, 0);
   	viewer.addPointCloud<PointWithRange>(veil_points_ptr, veil_points_color_handler, "veil points");
   	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
   	visualization::PointCloudColorHandlerCustom<PointWithRange> shadow_points_color_handler(shadow_points_ptr, 0, 255, 255);
   	viewer.addPointCloud<PointWithRange>(shadow_points_ptr, shadow_points_color_handler, "shadow points");
   	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");
  	
  	//
  	// Show points on range image
  	// 
  	visualization::RangeImageVisualizer* range_image_border_widget = 
//  		visualization::RangeImageVisualizer::getRangeImageBorderWidget(range_image, -std::numeric_limits<float>::infinity(),
//  			false, border_descriptions, "Range image with borders");
	    pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(range_image, -std::numeric_limits<float>::infinity (), 
	    			std::numeric_limits<float>::infinity (), false, border_descriptions, "Range image with borders");

  	//
  	// Main loop
  	// 
  	while (!viewer.wasStopped())
  	{
  		range_image_border_widget->spinOnce();
  		viewer.spinOnce();
  		pcl_sleep(0.01);
  	}
}