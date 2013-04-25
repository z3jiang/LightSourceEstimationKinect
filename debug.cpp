
#include "stdafx.h"
#include "debug.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <iomanip>
#include <sstream>

using namespace boost::posix_time;


namespace debug
{
	ptime start = microsec_clock::local_time();
	std::string getTime()
	{
		time_duration diff = microsec_clock::local_time() - start;

		int ms = diff.total_milliseconds();

		std::stringstream ss;
		ss << std::setfill('0') << setw(6) << ms / 1000;
		ss << '.' << setw(3) << (ms % 1000);
		return ss.str();
	}
}