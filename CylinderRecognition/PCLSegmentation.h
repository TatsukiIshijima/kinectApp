#include "stdafx.h"
#include "PCLPointType.h"
#include "PCLFilter.h"
#include <pcl\segmentation\sac_segmentation.h>
#include <pcl\segmentation\extract_clusters.h>

// 色付けセグメンテーション
void colored_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
// 平面セグメンテーション
pcl::PointIndices::Ptr planeSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr& cloud);
// 平面セグメンテーション
void planeSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, int iterations, double distanceThreshold);
std::vector < pcl::PointCloud<PointTypeRGB>::Ptr> planesSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr cloud, int iterations, double distanceThreshold, float k);
// 円柱セグメンテーション
void cylinderSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr& cloud, pcl::PointCloud<NormalType>::Ptr& normal, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, double weight, int iterarion, double distance, double min_radius, double max_radius);
// ユークリッドクラスタリング
std::vector<pcl::PointIndices> euclideanCluster(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointCloud<NormalType>::Ptr cloud_normal, float tolerance, double angle, int min_cluster_size);
std::vector < pcl::PointCloud<PointTypeRGB>::Ptr> newEuclideanCluster(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointCloud<NormalType>::Ptr cloud_normal, float tolerance, double angle, int min_cluster_size);
// 平面除去
void planeRemoval(pcl::PointCloud<PointTypeRGB>::Ptr cloud, int iterations, double distanceThreshold, float k);
// 平行平面除去
pcl::PointCloud<PointTypeRGB>::Ptr parallelPlaneRemove(pcl::PointCloud<PointTypeRGB>::Ptr cloud, int iterations, double distanceThreshold, float k, pcl::ModelCoefficients::Ptr floorPlaneCoeff);