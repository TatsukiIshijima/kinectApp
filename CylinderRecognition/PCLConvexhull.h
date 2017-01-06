#include "stdafx.h"
#include "PCLPointType.h"
#include <pcl\surface\convex_hull.h>

void perspectiveProjection(pcl::PointCloud<PointTypeRGB>::Ptr inputCloud, pcl::PointCloud<PointTypeRGB>::Ptr xy_cloud, pcl::PointCloud<PointTypeRGB>::Ptr yz_cloud, pcl::PointCloud<PointTypeRGB>::Ptr zx_cloud);
double calcConvexLength(pcl::PointCloud<PointTypeRGB>::Ptr cloud_vexhull);
void getConvexParam(pcl::PointCloud<PointTypeRGB>::Ptr inputCloud, pcl::PointCloud<PointTypeRGB>::Ptr cloud_vexhull, double *length, double *area);
double calcDoR(pcl::PointCloud<PointTypeRGB>::Ptr inputCloud);
