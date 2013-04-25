#pragma once

#include "stdafx.h"

#define DEBUG(arg) std::cout << debug::getTime() << ": " << arg

namespace debug
{
	std::string getTime();
}