#include "stdafx.h"
#include "PCLPointType.h"
#include "PCLFilter.h"

/* =========================================================================== */
/**
* @brief 一定範囲内の数でフィルターする
*
* @ param[in] cloud   点郡
* @ param[in] leaf    フィルターする範囲
*
*/
/* =========================================================================== */
void voxelGridFilter(pcl::PointCloud<PointTypeRGB>::Ptr cloud, float leaf)
{
	// フィルター前の点郡の数を表示する
	//pcl::console::print_info("before VoxelGridFilter point clouds : %d\n", cloud->size());

	pcl::VoxelGrid<PointTypeRGB> grid;

	// フィルターする範囲を設定
	grid.setLeafSize(leaf, leaf, leaf);

	// フィルターする点郡を設定
	grid.setInputCloud(cloud);

	// フィルター後の点郡を保存するためのクラウド
	pcl::PointCloud<PointTypeRGB>::Ptr filtered_cloud(new pcl::PointCloud<PointTypeRGB>);
	// グリッドフィルター実行
	grid.filter(*filtered_cloud);

	// フィルター後の点郡をコピー
	pcl::copyPointCloud(*filtered_cloud, *cloud);

	// フィルター後の点郡の数を表示する
	//pcl::console::print_info("after VoxelGridFilter point clouds : %d\n", cloud->size());
}

/* =========================================================================== */
/**
* @brief 一定範囲内の数でフィルターする
*
* @ param[in] cloud   点郡
* @ param[in] leaf    フィルターする範囲
*
*/
/* =========================================================================== */
void voxelGridFilterRGBNormal(pcl::PointCloud<PointTypeRGBNormal>::Ptr cloud_color_normal, float leaf)
{
	// フィルター前の点郡の数を表示する
	//pcl::console::print_info("before VoxelGridFilter point clouds : %d\n", cloud->size());

	pcl::VoxelGrid<PointTypeRGBNormal> grid;

	// フィルターする範囲を設定
	grid.setLeafSize(leaf, leaf, leaf);

	// フィルターする点郡を設定
	grid.setInputCloud(cloud_color_normal);

	// フィルター後の点郡を保存するためのクラウド
	pcl::PointCloud<PointTypeRGBNormal>::Ptr filtered_cloud(new pcl::PointCloud<PointTypeRGBNormal>);
	// グリッドフィルター実行
	grid.filter(*filtered_cloud);

	// フィルター後の点郡をコピー
	pcl::copyPointCloud(*filtered_cloud, *cloud_color_normal);

	// フィルター後の点郡の数を表示する
	//pcl::console::print_info("after VoxelGridFilter point clouds : %d\n", cloud->size());
}

/* =========================================================================== */
/**
* @brief 指定した次元(軸)について，値の範囲でフィルターする
*
* @ param[in] cloud      点郡
* @ param[in] axis       次元(軸)
* @ param[in] limit_min  範囲の上限値
* @ param[in] limit_max  範囲の下限値
* @ param[in] keep       organized点郡としてキープするか
*
*/
/* =========================================================================== */
void passThroughFilter(pcl::PointCloud<PointTypeRGB>::Ptr cloud, const std::string &axis, float limit_min, float limit_max, bool keep)
{
	// フィルター前の点郡の数を表示する
	//pcl::console::print_info("before PassThroughFilter point clouds : %d\n", cloud->size());

	pcl::PassThrough<PointTypeRGB> pass;

	// フィルターする点郡を設定
	pass.setInputCloud(cloud);
	pass.setKeepOrganized(keep);
	// フィルターする次元(軸)の設定
	pass.setFilterFieldName(axis);
	// フィルターする範囲の設定
	pass.setFilterLimits(limit_min, limit_max);

	// フィルター後の点郡を保存するためのクラウド
	pcl::PointCloud<PointTypeRGB>::Ptr filtered_cloud(new pcl::PointCloud<PointTypeRGB>);
	// グリッドフィルター実行
	pass.filter(*filtered_cloud);

	// フィルター後の点郡をコピー
	pcl::copyPointCloud(*filtered_cloud, *cloud);

	// フィルター後の点郡の数を表示する
	//pcl::console::print_info("after PassThroughFilter point clouds : %d\n", cloud->size());
}

/* =========================================================================== */
/**
* @brief 指定した次元(軸)について，値の範囲でフィルターする
*
* @ param[in] cloud      点郡
* @ param[in] axis       次元(軸)
* @ param[in] limit_min  範囲の上限値
* @ param[in] limit_max  範囲の下限値
* @ param[in] keep       organized点郡としてキープするか
*
*/
/* =========================================================================== */
void passThroughFilterRGBNormal(pcl::PointCloud<PointTypeRGBNormal>::Ptr cloud_color_normal, const std::string &axis, float limit_min, float limit_max, bool keep)
{
	// フィルター前の点郡の数を表示する
	//pcl::console::print_info("before PassThroughFilter point clouds : %d\n", cloud->size());

	pcl::PassThrough<PointTypeRGBNormal> pass;

	// フィルターする点郡を設定
	pass.setInputCloud(cloud_color_normal);
	pass.setKeepOrganized(keep);
	// フィルターする次元(軸)の設定
	pass.setFilterFieldName(axis);
	// フィルターする範囲の設定
	pass.setFilterLimits(limit_min, limit_max);

	// フィルター後の点郡を保存するためのクラウド
	pcl::PointCloud<PointTypeRGBNormal>::Ptr filtered_cloud(new pcl::PointCloud<PointTypeRGBNormal>);
	// グリッドフィルター実行
	pass.filter(*filtered_cloud);

	// フィルター後の点郡をコピー
	pcl::copyPointCloud(*filtered_cloud, *cloud_color_normal);

	// フィルター後の点郡の数を表示する
	//pcl::console::print_info("after PassThroughFilter point clouds : %d\n", cloud->size());
}

/* =========================================================================== */
/**
* @brief ノイズ除去フィルター
*
* @ param[in] cloud      点郡
* @ param[in] n          近傍点数
* @ param[in] sdm        標準偏差乗数(standard deviation multiplier)
*
*/
/* =========================================================================== */
void removalFilter(pcl::PointCloud<PointTypeRGB>::Ptr cloud, int n, float sdm)
{
	pcl::StatisticalOutlierRemoval<PointTypeRGB> sor;
	// フィルターする点郡を設定
	sor.setInputCloud(cloud);
	// 近傍点数の設定
	sor.setMeanK(n);
	// 標準偏差乗数の設定
	sor.setStddevMulThresh(sdm);
	// フィルター後の点郡を保存するためのクラウド
	pcl::PointCloud<PointTypeRGB>::Ptr filtered_cloud(new pcl::PointCloud<PointTypeRGB>);
	// フィルターの実行
	sor.filter(*filtered_cloud);
	// フィルター後の点郡をコピー
	pcl::copyPointCloud(*filtered_cloud, *cloud);
}

/* =========================================================================== */
/**
* @brief 入力点郡からインライア内のものまたはインライア外の点郡を抽出し除去する
*
* @ param[in] cloud      点郡
* @ param[in] inliers     インライア点郡(平面や円柱が検出された点郡)
* @ param[in] negative インライア内(true)かインライア外(false)
*/
/* =========================================================================== */
void extractIndices(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative)
{
	// 一時保管の点郡
	pcl::PointCloud<PointTypeRGB>::Ptr tmp(new pcl::PointCloud<PointTypeRGB>);
	// 点郡コピー
	pcl::copyPointCloud(*cloud, *tmp);

	// 点郡の抽出
	pcl::ExtractIndices<PointTypeRGB> extract;
	// 入力点郡の設定
	extract.setInputCloud(tmp);
	// 入力点郡から抽出する点郡の設定(インライア)
	extract.setIndices(inliers);

	// trueにするとinliers内の点郡除去,falseにするとinliers以外を除去
	extract.setNegative(negative);
	// フィルタの実行
	extract.filter(*cloud);
}

/* =========================================================================== */
/**
* @brief 入力法線点郡からインライア内のものまたはインライア外の法線点郡を抽出し除去する
*
* @ param[in] normal     法線点郡
* @ param[in] inliers       インライア点郡(平面や円柱が検出された点郡)
* @ param[in] negative   インライア内(true)かインライア外(false)
*/
/* =========================================================================== */
void extractIndiciesNormal(pcl::PointCloud<NormalType>::Ptr normal, pcl::PointIndices::Ptr inliers, bool negative)
{
	// 一時保管の法線点郡
	pcl::PointCloud<NormalType>::Ptr tmp(new pcl::PointCloud<NormalType>);
	// 法線点郡コピー
	pcl::copyPointCloud(*normal, *tmp);

	// 法線の抽出
	pcl::ExtractIndices<NormalType> extract_normal;
	// 入力法線の設定
	extract_normal.setInputCloud(tmp);
	// 入力法線から抽出する法線点郡の設定(インライア)
	extract_normal.setIndices(inliers);

	// trueにするとinliers内の法線除去,false にするとinliers以外の法線除去
	extract_normal.setNegative(negative);
	// 抽出実行
	extract_normal.filter(*normal);
}