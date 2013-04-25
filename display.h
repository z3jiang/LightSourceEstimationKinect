#pragma once

#include "stdafx.h"

class Display
{
public:
	Display(void);
	~Display(void);

	void run(PointCloudCPtr cloud);
};

