#pragma once

#include <Eigen/SVD>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <algorithm>

#include <boost/serialization/vector.hpp>
#include <boost/archive/xml_oarchive.hpp> 
#include <boost/archive/xml_iarchive.hpp>
#include <boost/filesystem.hpp>


#include <iostream>
#include <vector>
#include <cassert>
#include <stdexcept>

#include <boost/lexical_cast.hpp>

typedef pcl::PointXYZRGBA Point;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
typedef pcl::PointCloud<Point>::ConstPtr PointCloudCPtr;

typedef pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudNormalPtr;
typedef pcl::PointCloud<pcl::PointNormal>::ConstPtr PointCloudNormalCPtr;

