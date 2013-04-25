#include "stdafx.h"
#include "recorder.h"
#include "debug.h"

using namespace std;

Recorder::Recorder(void) :
    _counts(0)
{
}


Recorder::~Recorder(void)
{
}

void Recorder::run()
{
	pcl::Grabber* interface = new pcl::OpenNIGrabber();

	boost::function<void (const PointCloudCPtr&)> f =
		boost::bind(&Recorder::_callback, this, _1);

	interface->registerCallback(f);
	interface->start();

	while (_counts < 1)
	{
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}

	interface->stop();
}

void Recorder::_callback(const PointCloudCPtr &cloud)
{
	DEBUG("saving... ");
		DEBUG("Checking the cloud" << endl);

	PointCloudPtr validcloud (new pcl::PointCloud<Point>());
	vector<int> changes;
	pcl::removeNaNFromPointCloud<Point>(*cloud, *validcloud, changes);
	DEBUG("nan points removed: " << changes.size() << endl);

	pcl::io::savePCDFile<pcl::PointXYZRGBA>(
		boost::lexical_cast<string>(_counts) + ".pcd",
		*validcloud);

	DEBUG("file saved" << endl);
	_counts++;
}