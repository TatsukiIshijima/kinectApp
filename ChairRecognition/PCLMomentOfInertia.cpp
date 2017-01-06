#include "stdafx.h"
#include "PCLMomentOfInertia.h"

/* =========================================================================== */
/**
* @brief AABB(Axis-Aligned Bounding Box)による慣性モーメント推定
*
* @ param[in]   cloud      入力点郡
* @ param[in]   min_point  最小座標点
* @ param[in]   max_point  最大座標点
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
* @brief OBB(Oriented Bounding Box)による慣性モーメント推定
*
* @ param[in]   cloud              入力点郡
* @ param[in]   min_point          最小座標点
* @ param[in]   max_point          最大座標点
* @ param[in]   position_OBB	   立方体を表示する原点
* @ param[in]   rotational_matrix  クォータニオンベースの回転
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