#include "stdafx.h"
#include "PCLNormal.h"

/* =========================================================================== */
/**
* @brief 法線推定(点郡をクラスタリングしてその法線を算出する)
*
* @ param[in]     cloud               点郡
* @ param[out]    cloud_normals  法線点郡
*
*/
/* =========================================================================== */
pcl::PointCloud<NormalType>::Ptr normalEstimation(pcl::PointCloud<PointTypeRGB>::Ptr cloud)
{
	// 法線推定クラスを作成
	//pcl::NormalEstimation<PointType, NormalType> ne;
	pcl::NormalEstimationOMP<PointTypeRGB, NormalType> ne;
	// 点郡の設定
	ne.setInputCloud(cloud);

	// 探索アルゴリズムにkd-treeを設定
	pcl::search::KdTree<PointTypeRGB>::Ptr tree(new pcl::search::KdTree<PointTypeRGB>());
	ne.setSearchMethod(tree);

	// 法線点郡の準備
	pcl::PointCloud<NormalType>::Ptr cloud_normals(new pcl::PointCloud<NormalType>);

	// 近傍範囲の設定
	ne.setRadiusSearch(0.05);
	//ne.setKSearch(50);
	// 法線算出
	ne.compute(*cloud_normals);

	return cloud_normals;
}

/* =========================================================================== */
/**
* @brief 法線推定(点郡をクラスタリングしてその法線を算出する)
*
* @ param[in]     cloud               点郡
* @ param[out]    cloud_normals  法線点郡
*
*/
/* =========================================================================== */
pcl::PointCloud<NormalType>::Ptr normalEstimationKinect(pcl::PointCloud<PointTypeRGB>::Ptr cloud)
{
	// 法線推定クラスを作成
	//pcl::NormalEstimation<PointType, NormalType> ne;
	//pcl::NormalEstimationOMP<PointTypeRGB, NormalType> ne;
	pcl::IntegralImageNormalEstimation<PointTypeRGB, NormalType> ne;

	// 探索アルゴリズムにkd-treeを設定
	//pcl::search::KdTree<PointTypeRGB>::Ptr tree(new pcl::search::KdTree<PointTypeRGB>());
	//ne.setSearchMethod(tree);

	// 近傍範囲の設定
	//ne.setRadiusSearch(0.05);
	//ne.setKSearch(50);

	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);          // 計算方法の指定
	ne.setMaxDepthChangeFactor(0.01);                               // 計算対象の深さの変化の閾値
	ne.setNormalSmoothingSize(5.0);                                 // 法線の平滑化のサイズ

	// 法線点郡の準備
	pcl::PointCloud<NormalType>::Ptr cloud_normals(new pcl::PointCloud<NormalType>);

	// 点郡の設定
	ne.setInputCloud(cloud);
	// 法線算出
	ne.compute(*cloud_normals);

	return cloud_normals;
}
