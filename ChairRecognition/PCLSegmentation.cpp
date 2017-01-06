#include "stdafx.h"
#include "PCLPointType.h"
#include "PCLSegmentation.h"

/* =========================================================================== */
/**
* @brief 平面検出場所に色を付ける
*
* @ param[in] cloud      点郡
*
*/
/* =========================================================================== */
void colored_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	// モデルパラメータの定数
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//　許容範囲にある座標など(インライア)
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZRGB> sac_seg;
	// セグメンテーション条件設定
	sac_seg.setOptimizeCoefficients(true);
	// モデルの設定
	sac_seg.setModelType(pcl::SACMODEL_PLANE);
	// RANSACアルゴリズムの適応
	sac_seg.setMethodType(pcl::SAC_RANSAC);
	// 繰り返し回数(デフォルト50)
	sac_seg.setMaxIterations(100);
	// 距離閾値
	sac_seg.setDistanceThreshold(0.03);

	// セグメンテーションする点郡の設定
	sac_seg.setInputCloud(cloud);
	// セグメンテーション実行
	sac_seg.segment(*inliers, *coefficients);

	pcl::console::print_info("Model coefficients : %d\n", coefficients);

	// 検出した部分を赤に変更(inliersには点郡のインデックスが格納されている)
	for (size_t i = 0; i < inliers->indices.size(); i++) {
		cloud->points[inliers->indices[i]].r = 255;
		cloud->points[inliers->indices[i]].g = 0;
		cloud->points[inliers->indices[i]].b = 0;
	}
}

/* =========================================================================== */
/**
* @brief 平面検出
*
* @ param[in]     cloud      点郡
* @ param[out]   inliners   平面点郡のインデックス
*
*/
/* =========================================================================== */
pcl::PointIndices::Ptr planeSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr& cloud)
{
	// モデルパラメータの定数
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//　許容範囲にある座標など(インライア)
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<PointTypeRGB> sac_seg;
	// セグメンテーション条件設定
	sac_seg.setOptimizeCoefficients(true);
	// モデルの設定
	sac_seg.setModelType(pcl::SACMODEL_PLANE);
	// RANSACアルゴリズムの適応
	sac_seg.setMethodType(pcl::SAC_RANSAC);
	// 繰り返し回数(デフォルト50)
	sac_seg.setMaxIterations(100);
	// 距離閾値
	sac_seg.setDistanceThreshold(0.005);
	// セグメンテーションする点郡の設定
	sac_seg.setInputCloud(cloud);
	// セグメンテーション実行
	sac_seg.segment(*inliers, *coefficients);

	std::cerr << "Model coefficients : " << "a : " << coefficients->values[0]
		<< "  b : " << coefficients->values[1]
		<< "  c : " << coefficients->values[2]
		<< "  d : " << coefficients->values[3] << std::endl;

	return inliers;
}

/* =========================================================================== */
/**
* @brief 平面検出
*
* @ param[in]     cloud              入力点郡
* @ param[in]     coefficients       平面のパラメータを格納する変数
* @ param[in]     inliners           平面点郡のインデックス格納用変数
* @ param[in]     iterations         RANSACの繰り返し回数
* @ param[in]     distanceThreshold  距離の閾値
*
*/
/* =========================================================================== */
void planeSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, int iterations, double distanceThreshold)
{
	// 一時保管の点郡
	pcl::PointCloud<PointTypeRGB>::Ptr tmp(new pcl::PointCloud<PointTypeRGB>);
	// 点郡コピー
	pcl::copyPointCloud(*cloud, *tmp);

	pcl::SACSegmentation<PointTypeRGB> sac_seg;
	// セグメンテーション条件設定
	sac_seg.setOptimizeCoefficients(true);
	// モデルの設定
	sac_seg.setModelType(pcl::SACMODEL_PLANE);
	// RANSACアルゴリズムの適応
	sac_seg.setMethodType(pcl::SAC_RANSAC);
	// 繰り返し回数(デフォルト50)
	sac_seg.setMaxIterations(iterations);
	// 距離閾値
	sac_seg.setDistanceThreshold(distanceThreshold);
	// セグメンテーションする点郡の設定
	sac_seg.setInputCloud(tmp);
	// セグメンテーション実行
	sac_seg.segment(*inliers, *coefficients);
}

/* =========================================================================== */
/**
* @brief 複数平面検出
*
* @ param[in]     cloud              入力点郡
* @ param[in]     iterations         RANSACの繰り返し回数
* @ param[in]     distanceThreshold  距離の閾値
* @ param[in]     k                  入力点郡の数の定数倍より小さければループを終了させる定数
*
*/
/* =========================================================================== */
std::vector < pcl::PointCloud<PointTypeRGB>::Ptr> planesSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr cloud, int iterations, double distanceThreshold, float k)
{
	std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> planeClouds;

	pcl::PointCloud<PointTypeRGB>::Ptr inCloud(new pcl::PointCloud<PointTypeRGB>);
	pcl::PointCloud<PointTypeRGB>::Ptr outCloud(new pcl::PointCloud<PointTypeRGB>);

	int nr_points = (int)cloud->points.size();

	while (cloud->points.size() > k * nr_points)
	{
		pcl::copyPointCloud(*cloud, *inCloud);
		pcl::copyPointCloud(*cloud, *outCloud);

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// 平面検出
		planeSegmentation(cloud, coefficients, inliers, iterations, distanceThreshold);

		if (inliers->indices.size() == 0) break;

		// 点郡抽出
		extractIndices(inCloud, inliers, false);
		extractIndices(outCloud, inliers, true);

		pcl::PointCloud<PointTypeRGB>::Ptr planePointCloud(new pcl::PointCloud<PointTypeRGB>);
		*planePointCloud = *inCloud;
		planeClouds.push_back(planePointCloud);

		// 点郡更新
		*cloud = *outCloud;
	}
	return planeClouds;
}

/* =========================================================================== */
/**
* @brief 円柱検出
*
* @ param[in]     cloud     点郡
* @ param[in]     normal    法線点郡
* @ param[out]   inliners   円柱点郡のインデックス
*
*/
/* =========================================================================== */
void cylinderSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr& cloud, pcl::PointCloud<NormalType>::Ptr& normal, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, double weight, int iterarion, double distance, double min_radius, double max_radius)
{
	// モデルパラメータの定数
	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//　許容範囲にある座標など(インライア)
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentationFromNormals<PointTypeRGB, NormalType> sac_seg_norm;
	// セグメンテーション条件設定
	sac_seg_norm.setOptimizeCoefficients(true);
	// モデルの設定
	sac_seg_norm.setModelType(pcl::SACMODEL_CYLINDER);
	// RANSACアルゴリズムの適応
	sac_seg_norm.setMethodType(pcl::SAC_RANSAC);
	// (0~1の間で以下を設定)
	sac_seg_norm.setNormalDistanceWeight(weight);
	// 繰り返し回数
	sac_seg_norm.setMaxIterations(iterarion);
	// 距離閾値
	sac_seg_norm.setDistanceThreshold(distance);
	// 円柱の最大半径, 最小半径
	sac_seg_norm.setRadiusLimits(min_radius, max_radius);
	// セグメンテーションする点郡の設定
	sac_seg_norm.setInputCloud(cloud);
	// 法線点郡の設定
	sac_seg_norm.setInputNormals(normal);
	// セグメンテーション実行
	sac_seg_norm.segment(*inliers, *coefficients);
}

/* =========================================================================== */
/**
* @brief ユークリッドクラスタリング
*
* @ param[in]     cloud             点郡
* @ param[in]     normal            法線点郡
* @ param[in]     tolerance         許容差(x、y、z座標系)
* @ param[in]     angle             許容差(角度)
* @ param[in]     min_cluster_size  各クラスターの最小点郡数
* @ param[out]    cluster_indices   各クラスターのインデックス
*
*/
/* =========================================================================== */
std::vector<pcl::PointIndices> euclideanCluster(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointCloud<NormalType>::Ptr cloud_normal, float tolerance, double angle, int min_cluster_size)
{
	// kdtreeオブジェクト作成(抽出の探索法のため)
	boost::shared_ptr<pcl::KdTree<PointTypeRGB>> tree_ec(new pcl::KdTreeFLANN<PointTypeRGB>());
	tree_ec->setInputCloud(cloud);

	// 点郡と法線を用いてユークリッドクラスター抽出
	std::vector<pcl::PointIndices> cluster_indices;                                       // クラスターインデックス
	//const float tolerance = 0.02f;                                                        // 許容差(x、y、z座標系)
	//const double esp_angle = angle * (M_PI / 180.0);                                        // 許容差(角度？)
	double esp_angle = angle * (M_PI / 180.0);
	//const unsigned int min_cluster_size = 500;

	pcl::extractEuclideanClusters(*cloud, *cloud_normal, tolerance, tree_ec, cluster_indices, esp_angle, min_cluster_size);
	pcl::EuclideanClusterExtraction<PointTypeRGB> ec;

	return cluster_indices;
}

/* =========================================================================== */
/**
* @brief ユークリッドクラスタリング改良版
*        各クラスター点郡で返すように改良
*
* @ param[in]     cloud             点郡
* @ param[in]     normal            法線点郡
* @ param[in]     tolerance         許容差(x、y、z座標系)
* @ param[in]     angle             許容差(角度)
* @ param[in]     min_cluster_size  各クラスターの最小点郡数
* @ param[out]    clusterClouds     各クラスター点郡
*
*/
/* =========================================================================== */
std::vector < pcl::PointCloud<PointTypeRGB>::Ptr> newEuclideanCluster(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointCloud<NormalType>::Ptr cloud_normal, float tolerance, double angle, int min_cluster_size)
{
	// kdtreeオブジェクト作成(抽出の探索法のため)
	boost::shared_ptr<pcl::KdTree<PointTypeRGB>> tree_ec(new pcl::KdTreeFLANN<PointTypeRGB>());
	tree_ec->setInputCloud(cloud);

	// 点郡と法線を用いてユークリッドクラスター抽出
	std::vector<pcl::PointIndices> cluster_indices;                                       // クラスターインデックス
	std::vector < pcl::PointCloud<PointTypeRGB>::Ptr > clusterClouds;
	double esp_angle = angle * (M_PI / 180.0);

	pcl::extractEuclideanClusters(*cloud, *cloud_normal, tolerance, tree_ec, cluster_indices, esp_angle, min_cluster_size);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
		pcl::PointCloud<PointTypeRGB>::Ptr cloud_cluster(new pcl::PointCloud<PointTypeRGB>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
			cloud_cluster->points.push_back(cloud->points[*pit]);
		}
		// 各クラスター点郡を動的配列に保存
		clusterClouds.push_back(cloud_cluster);
	}
	return clusterClouds;
}

/* =========================================================================== */
/**
* @brief 平面除去
*
* @ param[in]     cloud				 入力点郡
* @ param[in]     iterations         RANSACの繰り返し回数
* @ param[in]     distanceThreshold  距離の閾値
* @ param[in]     k                  入力点郡の数の定数倍より小さければループを終了させる定数
*/
/* =========================================================================== */
void planeRemoval(pcl::PointCloud<PointTypeRGB>::Ptr cloud, int iterations, double distanceThreshold, float k)
{
	pcl::PointCloud<PointTypeRGB>::Ptr out_cloud(new pcl::PointCloud<PointTypeRGB>);

	int nr_points = (int)cloud->points.size();
	while (cloud->points.size() > k * nr_points)
	{
		pcl::copyPointCloud(*cloud, *out_cloud);

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		planeSegmentation(cloud, coefficients, inliers, iterations, distanceThreshold);

		if (inliers->indices.size() == 0) break;

		extractIndices(out_cloud, inliers, true);
		*cloud = *out_cloud;
	}
}

/* =========================================================================== */
/**
* @brief 床と平行な面を除去する
*
* @ param[in]     cloud					入力点郡
* @ param[in]     iterations			RANSACの繰り返し回数
* @ param[in]     distanceThreshold		距離の閾値
* @ param[in]	  k						入力点郡の数の定数倍より小さければループを終了させる定数
* @ param[in]	  floorPlaneCoeff		床の法線ベクトル
*
*/
/* =========================================================================== */
pcl::PointCloud<PointTypeRGB>::Ptr parallelPlaneRemove(pcl::PointCloud<PointTypeRGB>::Ptr cloud, int iterations, double distanceThreshold, float k, pcl::ModelCoefficients::Ptr floorPlaneCoeff)
{
	pcl::PointCloud<PointTypeRGB>::Ptr copyCloud(new pcl::PointCloud<PointTypeRGB>);
	pcl::PointCloud<PointTypeRGB>::Ptr outCloud(new pcl::PointCloud<PointTypeRGB>);
	pcl::PointCloud<PointTypeRGB>::Ptr removedCloud(new pcl::PointCloud<PointTypeRGB>);

	pcl::copyPointCloud(*cloud, *copyCloud);
	pcl::copyPointCloud(*cloud, *removedCloud);

	int nr_points = (int)copyCloud->points.size();

	while (copyCloud->points.size() > k * nr_points)
	{
		pcl::copyPointCloud(*copyCloud, *outCloud);

		pcl::ModelCoefficients::Ptr planeCoeff(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// 平面検出
		planeSegmentation(copyCloud, planeCoeff, inliers, iterations, distanceThreshold);

		if (inliers->indices.size() == 0) break;

		// 内積計算
		double innerPro = planeCoeff->values[0] * floorPlaneCoeff->values[0] +
						  planeCoeff->values[1] * floorPlaneCoeff->values[1] +
						  planeCoeff->values[2] * floorPlaneCoeff->values[2];

		// 内積値の閾値判定
		if (innerPro >= 0.8 || innerPro <= -0.8) extractIndices(removedCloud, inliers, true);

		// 点郡抽出
		extractIndices(outCloud, inliers, true);
		// 点郡更新
		*copyCloud = *outCloud;
	}
	return removedCloud;
}