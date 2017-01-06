#include "stdafx.h"
#include <vector>
#include "PCLPointType.h"
#include <pcl/octree/octree.h>

pcl::PointCloud<PointTypeRGB>::Ptr VoxelSearch(pcl::PointCloud<PointTypeRGB>::Ptr cloud, float resolution, pcl::PointXYZRGB searchPoint);
pcl::PointCloud<PointTypeRGB>::Ptr KNNSearch(pcl::PointCloud<PointTypeRGB>::Ptr cloud, float resolution, pcl::PointXYZRGB searchPoint, int K);
pcl::PointCloud<PointTypeRGB>::Ptr RadiusSearch(pcl::PointCloud<PointTypeRGB>::Ptr cloud, float resolution, pcl::PointXYZRGB searchPoint, float radius);
pcl::PointCloud<PointTypeRGB>::Ptr diffExtraction(pcl::PointCloud<PointTypeRGB>::Ptr baseCloud, pcl::PointCloud<PointTypeRGB>::Ptr testCloud, double resolution);