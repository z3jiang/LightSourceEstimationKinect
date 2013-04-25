#pragma once

#include "stdafx.h"


class SurfaceNormal
{
public:
	SurfaceNormal(void);
	~SurfaceNormal(void);

	PointCloudNormalPtr run(PointCloudCPtr cloud);

};

