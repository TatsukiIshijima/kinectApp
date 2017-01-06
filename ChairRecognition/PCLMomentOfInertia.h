#include "stdafx.h"
#include "PCLPointType.h"
#include <pcl\features\/moment_of_inertia_estimation.h>

void MomentOfInertiaByAABB(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointXYZRGB* min_point, pcl::PointXYZRGB* max_point);
void MomentOfInertiaByOBB(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointXYZRGB* min_point, pcl::PointXYZRGB* max_point, pcl::PointXYZRGB* position_OBB, Eigen::Matrix3f* rotational_matrix);