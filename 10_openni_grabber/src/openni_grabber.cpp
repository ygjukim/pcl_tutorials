#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

using namespace pcl;

class SimpleOpenNIProcessor
{
public:
	void cloud_cb_(const PointCloud<PointXYZRGBA>::ConstPtr &cloud)
	{
		static unsigned count = 0;
		static double last = pcl::getTime();
		if (++count == 30)
		{
			double now = pcl::getTime();
			std::cout << "distance of center pixel: " << cloud->points[(cloud->width >> 1) * (cloud->height + 1)].z
					  << " mm. Average framerate: " << double(count)/double(now - last) << " Hz.\n";
			count = 0;
			last = now;
		}
	}

	void run()
	{
		Grabber* interface = new OpenNIGrabber();

		boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f = 
			boost::bind(&SimpleOpenNIProcessor::cloud_cb_, this, _1);

		boost::signals2::connection c = interface->registerCallback(f);

		interface->start();

		while (1) 
		{
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}

		interface->stop();			
	}
};

int main()
{
	SimpleOpenNIProcessor v;
	v.run();
	return 0;
}
