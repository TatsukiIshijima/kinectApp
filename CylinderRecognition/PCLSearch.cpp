#include "stdafx.h"
#include "PCLPointType.h"
#include "PCLSearch.h"

/* =========================================================================== */
/**
* @brief Neighbors within Voxel Search
*
* @ param[in]     cloud          �_�S
* @ param[in]     resolution     �{�N�Z�������H
* @ param[in]     searchPoint    �T����_
* @ param[out]    outCloud       �T���_�S
*
*/
/* =========================================================================== */
pcl::PointCloud<PointTypeRGB>::Ptr VoxelSearch(pcl::PointCloud<PointTypeRGB>::Ptr cloud, float resolution, pcl::PointXYZRGB searchPoint)
{
	pcl::PointCloud<PointTypeRGB>::Ptr outCloud(new pcl::PointCloud<PointTypeRGB>);

	// Octree�̃C���X�^���X
	pcl::octree::OctreePointCloudSearch<PointTypeRGB> octree(resolution);
	// �_�S����
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	std::vector<int> pointIdxVec;

	if (octree.voxelSearch(searchPoint, pointIdxVec))
	{
		for (size_t i = 0; i < pointIdxVec.size(); i++)
		{
			outCloud->push_back(cloud->points[pointIdxVec[i]]);
		}
	}

	return outCloud;
}

/* =========================================================================== */
/**
* @brief K Nearest Neighbor Search
*
* @ param[in]     cloud          �_�S
* @ param[in]     resolution     �{�N�Z�������H
* @ param[in]     searchPoint    �T����_
* @ param[in]     K              �ߖT�_��
* @ param[out]    outCloud       �T���_�S
*
*/
/* =========================================================================== */
pcl::PointCloud<PointTypeRGB>::Ptr KNNSearch(pcl::PointCloud<PointTypeRGB>::Ptr cloud, float resolution, pcl::PointXYZRGB searchPoint, int K)
{
	pcl::PointCloud<PointTypeRGB>::Ptr outCloud(new pcl::PointCloud<PointTypeRGB>);

	// Octree�̃C���X�^���X
	pcl::octree::OctreePointCloudSearch<PointTypeRGB> octree(resolution);
	// �_�S����
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	std::vector<int> pointIdxKNNSearch;
	std::vector<float> pointKNNSquaredDistance;

	if (octree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxKNNSearch.size(); i++)
		{
			outCloud->push_back(cloud->points[pointIdxKNNSearch[i]]);
		}
	}
	return outCloud;
}

/* =========================================================================== */
/**
* @brief Neighbors within Radius Search
*
* @ param[in]     cloud          �_�S
* @ param[in]     resolution     �{�N�Z�������H
* @ param[in]     searchPoint    �T����_
* @ param[in]     redius         ���a
* @ param[out]    outCloud       �T���_�S
*
*/
/* =========================================================================== */
pcl::PointCloud<PointTypeRGB>::Ptr RadiusSearch(pcl::PointCloud<PointTypeRGB>::Ptr cloud, float resolution, pcl::PointXYZRGB searchPoint, float radius)
{
	pcl::PointCloud<PointTypeRGB>::Ptr outCloud(new pcl::PointCloud<PointTypeRGB>);

	// Octree�̃C���X�^���X
	pcl::octree::OctreePointCloudSearch<PointTypeRGB> octree(resolution);
	// �_�S����
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
		{
			outCloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
		}
	}
	return outCloud;
}

/* =========================================================================== */
/**
* @brief �_�S����
*
* @ param[in]     baseCloud       ��_�S
* @ param[in]     testCloud		  ��r�_�S     
* @ param[in]     resolution      �{�N�Z�������H
* @ param[out]    diffCloud       �����_�S
*
*/
/* =========================================================================== */
pcl::PointCloud<PointTypeRGB>::Ptr diffExtraction(pcl::PointCloud<PointTypeRGB>::Ptr baseCloud, pcl::PointCloud<PointTypeRGB>::Ptr testCloud, double resolution)
{
	pcl::octree::OctreePointCloudChangeDetector<PointTypeRGB> octree(resolution);
	octree.setInputCloud(baseCloud);
	octree.addPointsFromInputCloud();

	octree.switchBuffers();

	octree.setInputCloud(testCloud);
	octree.addPointsFromInputCloud();

	std::vector<int> newPointIdxVector;

	octree.getPointIndicesFromNewVoxels(newPointIdxVector);

	pcl::PointCloud<PointTypeRGB>::Ptr diffCloud(new pcl::PointCloud<PointTypeRGB>);

	diffCloud->width = baseCloud->points.size() + testCloud->points.size();
	diffCloud->height = 1;
	diffCloud->points.resize(diffCloud->width * diffCloud->height);

	int n = 0;
	for (size_t i = 0; i < newPointIdxVector.size(); i++)
	{
		diffCloud->points[i].x = testCloud->points[newPointIdxVector[i]].x;
		diffCloud->points[i].y = testCloud->points[newPointIdxVector[i]].y;
		diffCloud->points[i].z = testCloud->points[newPointIdxVector[i]].z;
		//diffCloud->points[i].r = testCloud->points[newPointIdxVector[i]].r;
		//diffCloud->points[i].g = testCloud->points[newPointIdxVector[i]].g;
		//diffCloud->points[i].b = testCloud->points[newPointIdxVector[i]].b;
		n++;
	}

	diffCloud->width = n;
	diffCloud->height = 1;
	diffCloud->points.resize(diffCloud->width * diffCloud->height);

	return diffCloud;
}