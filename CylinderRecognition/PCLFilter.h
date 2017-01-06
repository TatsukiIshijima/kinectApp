#include "stdafx.h"
#include "PCLPointType.h"
#include <pcl\filters\voxel_grid.h>
#include <pcl\filters\passthrough.h>
#include <pcl\filters\statistical_outlier_removal.h>
#include <pcl\filters\extract_indices.h>

// VoxelGridFilter
void voxelGridFilter(pcl::PointCloud<PointTypeRGB>::Ptr cloud, float leaf);
void voxelGridFilterRGBNormal(pcl::PointCloud<PointTypeRGBNormal>::Ptr cloud_color_normal, float leaf);
// PassThroughFilter
void passThroughFilter(pcl::PointCloud<PointTypeRGB>::Ptr cloud, const std::string &axis, float limit_min, float limit_max, bool keep);
void passThroughFilterRGBNormal(pcl::PointCloud<PointTypeRGBNormal>::Ptr cloud_color_normal, const std::string &axis, float limit_min, float limit_max, bool keep);
// RemovalFilter
void removalFilter(pcl::PointCloud<PointTypeRGB>::Ptr cloud, int n, float sdm);
// 点郡抽出
void extractIndices(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative);
// 法線抽出
void extractIndiciesNormal(pcl::PointCloud<NormalType>::Ptr normal, pcl::PointIndices::Ptr inliers, bool negative);
