#pragma once

#include "stdafx.h"
#include <Eigen/Core>

class LSEstimation
{
public:
	LSEstimation(bool enableAmbientCancellation) : _ac(enableAmbientCancellation) {};
	~LSEstimation(void);

	Eigen::Vector3f run(PointCloudCPtr cloud, PointCloudNormalPtr normals);


private:
	void getSeg(
		PointCloudCPtr cloud, PointCloudNormalPtr normals, vector< vector<int> > &indicies);

	Eigen::Vector3f estimateLS(
		PointCloudCPtr cloud, PointCloudNormalCPtr normals, vector<int> &indicies);

	bool _ac;
};

