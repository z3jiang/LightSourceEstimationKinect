#pragma once

#include "stdafx.h"


class Loader
{
public:
	Loader(void);
	~Loader(void);

	PointCloudPtr load();
	void binarize();
};

