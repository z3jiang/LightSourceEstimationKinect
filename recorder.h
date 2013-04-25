#pragma once

#include "stdafx.h"

class Recorder
{
public:
	Recorder(void);
	~Recorder(void);

	void run();

private:
	void _callback(const PointCloudCPtr &cloud);

	int _counts;
};

