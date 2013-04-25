

#include "stdafx.h"
#include "openni_grabber.h"
#include "debug.h"

using namespace std;



void SimpleOpenNIViewer::run()
{
	pcl::Grabber* interface = new pcl::OpenNIGrabber();

	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
		boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

	interface->registerCallback (f);

	interface->start ();

	while (!viewer.wasStopped())
	{
		boost::this_thread::sleep (boost::posix_time::seconds (1));
	}

	DEBUG("exiting" << endl);
	interface->stop ();
}

void SimpleOpenNIViewer::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
	if (!viewer.wasStopped())
		viewer.showCloud (cloud);
}
