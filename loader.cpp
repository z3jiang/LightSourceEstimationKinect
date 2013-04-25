#include "stdafx.h"
#include "loader.h"
#include "debug.h"
#include <boost/filesystem.hpp>

using namespace std;

Loader::Loader(void)
{
}


Loader::~Loader(void)
{
}

PointCloudPtr Loader::load()
{
	DEBUG("loading pcd file " << endl);

	PointCloudPtr cloud(new pcl::PointCloud<Point>());
	cloud->resize(640*480);

	int success;
	
	if (boost::filesystem::exists("0.b.pcd"))
	{
		DEBUG("loading binary version" << endl);
		success = pcl::io::loadPCDFile<Point>("0.b.pcd", *cloud);
	}
	else
	{
		DEBUG("loading ascii version" << endl);
		success = pcl::io::loadPCDFile<Point>("0.pcd", *cloud);
	}
	
	if (success < 0)
	{
		throw runtime_error("failed to load pcd");
	}

	return cloud;
}


void Loader::binarize()
{
	if (boost::filesystem::exists("0.b.pcd"))
	{
		DEBUG("already binarized" << endl);
		return;
	}

	PointCloudPtr orig = load();

	DEBUG("converting to binary" << endl);
	int success = pcl::io::savePCDFileBinary<Point>("0.b.pcd", *orig);

	if (success < 0)
	{
		throw runtime_error("failed to load pcd");
	}
}