#include "stdafx.h"
#include "PCLMomentOfInertia.h"

/* =========================================================================== */
/**
* @brief AABB(Axis-Aligned Bounding Box)�ɂ�銵�����[�����g����
*
* @ param[in]   cloud      ���͓_�S
* @ param[in]   min_point  �ŏ����W�_
* @ param[in]   max_point  �ő���W�_
*
*/
/* =========================================================================== */
void MomentOfInertiaByAABB(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointXYZRGB* min_point, pcl::PointXYZRGB* max_point)
{
	pcl::MomentOfInertiaEstimation <PointTypeRGB> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getAABB(*min_point, *max_point);
}

/* =========================================================================== */
/**
* @brief OBB(Oriented Bounding Box)�ɂ�銵�����[�����g����
*
* @ param[in]   cloud              ���͓_�S
* @ param[in]   min_point          �ŏ����W�_
* @ param[in]   max_point          �ő���W�_
* @ param[in]   position_OBB	   �����̂�\�����錴�_
* @ param[in]   rotational_matrix  �N�H�[�^�j�I���x�[�X�̉�]
*
*/
/* =========================================================================== */
void MomentOfInertiaByOBB(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointXYZRGB* min_point, pcl::PointXYZRGB* max_point, pcl::PointXYZRGB* position_OBB, Eigen::Matrix3f* rotational_matrix)
{
	pcl::MomentOfInertiaEstimation <PointTypeRGB> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getOBB(*min_point, *max_point, *position_OBB, *rotational_matrix);
}