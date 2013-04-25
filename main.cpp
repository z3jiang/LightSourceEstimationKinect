
#include "stdafx.h"
#include "recorder.h"
#include "display.h"
#include "debug.h"
#include "loader.h"
#include "lsestimation.h"
#include "surfacenormal.h"
#include "openni_grabber.h"

#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;

int realmain(int ac, char** av)
{
	try
	{
		po::options_description desc("Allowed options");
		desc.add_options()
			("help", "produce help message")
			("r", "record mode")
			("c", "convert to binary mode")
			("d", "display mode. requires recorded pcd in folder")
			("n", "calculate surface normal")
			("e", "estimate lightsource")
			("v", "view kinect data online")
			;

		po::variables_map vm;
		po::store(po::parse_command_line(ac, av, desc), vm);
		po::notify(vm);    

		if (vm.count("help"))
		{
			cout << desc << endl;
			return 1;
		}

		if (vm.count("r"))
		{
			DEBUG("recording mode" << endl);
			Recorder().run();
		}
		else if (vm.count("c"))
		{
			Loader().binarize();
		}
		else if (vm.count("d"))
		{
			DEBUG("display mode" << endl);
			Display().run( Loader().load() );
		}
		else if (vm.count("n"))
		{
			SurfaceNormal().run( Loader().load() );
		}
		else if (vm.count("e"))
		{
			Loader().binarize();
			DEBUG("estimating lightsource" << endl);
			PointCloudPtr points = Loader().load();
			PointCloudNormalPtr normals = SurfaceNormal().run(points);
			Eigen::Vector3f ls = LSEstimation().run(points, normals);
			cout << "light source: " << ls << endl;
		}
		else if (vm.count("v"))
		{
			SimpleOpenNIViewer().run();
		}
		else
		{
			cerr << "No mode specified" << endl;
			return 1;
		}

	}
	catch (exception& e)
	{
		cerr << "Unexpected exception" << endl;
		cerr << e.what() << endl;
		return 1;
	}

	return 0;
}

int main(int ac, char** av)
{
	int ret = realmain(ac, av);
	cout << "program exited. hit enter to quit" << endl;
	cin.get();
	return ret;
}