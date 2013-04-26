#include "stdafx.h"
#include "lsestimation.h"
#include "debug.h"

using namespace std;


LSEstimation::~LSEstimation(void)
{
}

char lastKey = 0;

void _kb(const pcl::visualization::KeyboardEvent& evt, void* junk)
{
	if (!evt.keyDown())
	{
		return;
	}

	lastKey = evt.getKeyCode();
}

void LSEstimation::getSeg(
	PointCloudCPtr cloud, PointCloudNormalPtr normals, vector< vector<int> > &indicies)
{
	if (boost::filesystem::exists("0.seg.txt"))
	{
		DEBUG("loading from cached segmentation" << endl);

		std::ifstream file("0.seg.txt"); 
		boost::archive::xml_iarchive ia(file); 
		ia >> BOOST_SERIALIZATION_NVP(indicies);

		DEBUG("loaded " << indicies.size() << " clusters" << endl);
	}
	else
	{
		DEBUG("not cached. computating segmentation" <<endl);

		pcl::IndicesPtr indices (new std::vector <int>);
		pcl::PassThrough<Point> pass;
		pass.setInputCloud (cloud);
		pass.filter (*indices);

		pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);

		pcl::RegionGrowingRGB<Point, pcl::PointNormal> reg;
		reg.setInputCloud (cloud);
		reg.setInputNormals(normals);
		reg.setIndices (indices);
		reg.setSearchMethod (tree);
		reg.setDistanceThreshold (100);
		reg.setPointColorThreshold (4);
		reg.setRegionColorThreshold (8);
		reg.setMinClusterSize (600);

		std::vector <pcl::PointIndices> *clusters = new std::vector<pcl::PointIndices>;
		reg.extract (*clusters);

		DEBUG("found " << clusters->size() << " clusters" << endl);

		pcl::PointCloud<Point>::Ptr colored_cloud = reg.getColoredCloudRGBA();

		pcl::visualization::CloudViewer v1 ("Cluster viewer");
		v1.showCloud (colored_cloud);
		lastKey = 0;
		v1.registerKeyboardCallback (_kb);
		
		while (lastKey == 0)
		{
			boost::this_thread::sleep (boost::posix_time::microseconds (1000));
		}

		for (size_t i=0; i<clusters->size(); i++)
		{
			lastKey = 0;
			
			pcl::PointCloud<Point>::Ptr subcloud (new pcl::PointCloud<Point>);
			pcl::ExtractIndices<Point> extract;
			extract.setIndices(pcl::PointIndices::Ptr(new pcl::PointIndices(clusters->at(i))));
			extract.setInputCloud(cloud);
			extract.filter(*subcloud);

			v1.showCloud(subcloud);

			while (true)
			{
				if (lastKey == 'y')
				{
					DEBUG("using this segment" << endl);
					indicies.push_back( clusters->at(i).indices );
					break;
				}
				else if (lastKey == 'n')
				{
					DEBUG("not using this segment" << endl);
					break;
				}
				else
				{
					boost::this_thread::sleep (boost::posix_time::milliseconds(100));
				}
			}

		}
		
		DEBUG("user decided to keep " << indicies.size() << " segments" << endl);
		DEBUG("caching segmentaiton results" << endl);
		std::ofstream file("0.seg.txt"); 
		boost::archive::xml_oarchive oa(file);
		oa & BOOST_SERIALIZATION_NVP(indicies);
		file.close();
		DEBUG("cached" << endl);
	}
}

Eigen::Vector3f LSEstimation::run(PointCloudCPtr cloud, PointCloudNormalPtr normals)
{
    if (cloud->size() != normals->size())
	{
		throw runtime_error("bad args: " + 
			boost::lexical_cast<string>(cloud->size()) + " rgbxyz points " + 
			boost::lexical_cast<string>(normals->size()) + " normals");
	}

	vector< vector<int> > indicies;
	getSeg(cloud, normals, indicies);

	ofstream ofile("0.results.txt");

	DEBUG("starting estimations " << endl);
	Eigen::Vector3f total = Eigen::Vector3f::Zero();
	for (size_t i=0; i<indicies.size(); i++)
	{
		Eigen::Vector3f ls = estimateLS(cloud, normals, indicies[i]);
		ofile << "Segment " << i << endl;
		ofile << ls << endl << endl;

		total = total + ls / indicies.size();
	}

	DEBUG("combined: " << total << endl);
	total.normalize();
	DEBUG("final: " << total << endl);
	ofile << "final: " << endl;
	ofile << total << endl;
	ofile.close();
	cin.get();
	exit(0);
}


Eigen::Vector3f LSEstimation::estimateLS(
	PointCloudCPtr cloud, PointCloudNormalCPtr normals, vector<int> &indicies)
{
	size_t n = indicies.size();
	DEBUG("estimating LS location from " << n << " points" << endl);
	DEBUG("ambient cancellation: " << _ac << endl);
	const static int limit = 7000;
	int times = indicies.size()/limit;

	Eigen::Vector3f ret = Eigen::Vector3f::Zero();

	for (int i=0; i<=times; i++)
	{
		DEBUG("subsample iteration " << i << "/" << times << endl);
		// too big, sub sample
		vector<int> current = indicies;
		std::random_shuffle(current.begin(), current.end());
		current.resize(limit);
		sort(current.begin(), current.end());
		DEBUG("resizing down to " << limit << endl);
		n = limit;

		Eigen::MatrixX3f A(n, 3);
		Eigen::VectorXf b(n);
		
		float totalR = 0;

		for (size_t i=0; i<n; i++)
		{
			pcl::PointNormal n =  normals->at(indicies[i]);
			A(i, 0) =  n.normal[0];
			A(i, 1) =  n.normal[1];
			A(i, 2) =  n.normal[2];

			Point p = cloud->at(indicies[i]);
			float r =  0.2126*p.r + 0.7152*p.g + 0.0722*p.b;
			totalR += r;
			b(i) = r + n.normal[0] * p.x + n.normal[1] * p.y + n.normal[2] * p.z;

		}

		if (_ac)
		{
			for (size_t i=0; i<n; i++)
			{
				b(i) -= totalR/n*0.05;
				if (b(i) < 0)
				{
					b(i) = 0;
				}
			}
		}

		DEBUG("SVD..." << endl);
		Eigen::JacobiSVD<Eigen::MatrixX3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::VectorXf L = svd.solve(b);
		L.normalize();

		ret += L;
	}

	ret.normalize();

	DEBUG("estimated ls: " << endl << ret << endl << endl);
	return ret;
}