#include "stdafx.h"
#include "surfacenormal.h"
#include "debug.h"
#include "loader.h"

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <boost/filesystem.hpp>


using namespace std;

SurfaceNormal::SurfaceNormal(void)
{
}


SurfaceNormal::~SurfaceNormal(void)
{
}

void visualize(PointCloudCPtr cloud, PointCloudNormalPtr normals)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<Point> rgb(cloud);
    viewer->addPointCloud<Point> (cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	
	viewer->addPointCloudNormals<Point,pcl::PointNormal>(cloud, normals);
	viewer->initCameraParameters ();
	
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce();
		boost::this_thread::sleep (boost::posix_time::microseconds (100));
	}
}


PointCloudNormalPtr SurfaceNormal::run(PointCloudCPtr cloud)
{
	DEBUG("surface normal estimation of " << cloud->size() << " points" << endl);

	PointCloudNormalPtr normals(new pcl::PointCloud<pcl::PointNormal>);

	if (boost::filesystem::exists("0.n.b.pcb"))
	{
		DEBUG("trying to load from cache" << endl);
		int success = pcl::io::loadPCDFile<pcl::PointNormal>("0.n.b.pcb", *normals);
		if (success >= 0)
		{
			DEBUG("successfully loaded from cache" << endl);
			visualize(cloud, normals);
			return normals;
		}
	}


	pcl::NormalEstimation<Point, pcl::PointNormal> ne;
	ne.setInputCloud (cloud);

	pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (0.03);

	ne.compute (*normals);

	DEBUG("normals calculated " << normals->size() << " points" << endl);


	DEBUG("caching results" << endl);
	pcl::io::savePCDFileBinary<pcl::PointNormal>("0.n.b.pcb", *normals);

	visualize(cloud, normals);

	return normals;
}