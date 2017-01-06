#include "stdafx.h"
#include "PCLPointType.h"
#include "PCLFilter.h"
#include <pcl\segmentation\sac_segmentation.h>
#include <pcl\segmentation\extract_clusters.h>

// �F�t���Z�O�����e�[�V����
void colored_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
// ���ʃZ�O�����e�[�V����
pcl::PointIndices::Ptr planeSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr& cloud);
// ���ʃZ�O�����e�[�V����
void planeSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, int iterations, double distanceThreshold);
std::vector < pcl::PointCloud<PointTypeRGB>::Ptr> planesSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr cloud, int iterations, double distanceThreshold, float k);
// �~���Z�O�����e�[�V����
void cylinderSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr& cloud, pcl::PointCloud<NormalType>::Ptr& normal, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, double weight, int iterarion, double distance, double min_radius, double max_radius);
// ���[�N���b�h�N���X�^�����O
std::vector<pcl::PointIndices> euclideanCluster(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointCloud<NormalType>::Ptr cloud_normal, float tolerance, double angle, int min_cluster_size);
std::vector < pcl::PointCloud<PointTypeRGB>::Ptr> newEuclideanCluster(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointCloud<NormalType>::Ptr cloud_normal, float tolerance, double angle, int min_cluster_size);
// ���ʏ���
void planeRemoval(pcl::PointCloud<PointTypeRGB>::Ptr cloud, int iterations, double distanceThreshold, float k);
// ���s���ʏ���
pcl::PointCloud<PointTypeRGB>::Ptr parallelPlaneRemove(pcl::PointCloud<PointTypeRGB>::Ptr cloud, int iterations, double distanceThreshold, float k, pcl::ModelCoefficients::Ptr floorPlaneCoeff);