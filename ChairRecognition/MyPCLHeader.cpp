#include "stdafx.h"
#include "MyPCLHeader.h"

/* =========================================================================== */
/**
* @brief 各点郡に色を付ける
*
* @ param[in]     viewer         ビューアー
* @ param[in]     clusterClouds  点郡配列
* @ param[in]     id             描画用ID
* @ param[in]	  v				 描画場所
*
*/
/* =========================================================================== */
void drawClusterPointCloud(pcl::visualization::PCLVisualizer& viewer, std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> clusterClouds, std::string id, int v)
{
	int colors[7][3] = { { 255,   0,   0 },		// 赤
						 {   0, 255,   0 },		// 緑
						 {   0,   0, 255 },		// 青
						 { 255,   0, 125 },		// ピンク
						 { 255, 255,   0 },		// 黄
						 { 255, 125,   0 },		// オレンジ
						 { 0, 255, 255 } };   // 水色
	for (int i = 0; i < clusterClouds.size(); i++)
	{
		int num = i % 7;
		pcl::visualization::PointCloudColorHandlerCustom<PointTypeRGB> color(clusterClouds[i], colors[num][0], colors[num][1], colors[num][2]);

		switch (num)
		{
		case 0:
			viewer.addPointCloud(clusterClouds[i], color, id + std::to_string(i), v);
			break;
		case 1:
			viewer.addPointCloud(clusterClouds[i], color, id + std::to_string(i), v);
			break;
		case 2:
			viewer.addPointCloud(clusterClouds[i], color, id + std::to_string(i), v);
			break;
		case 3:
			viewer.addPointCloud(clusterClouds[i], color, id + std::to_string(i), v);
			break;
		case 4:
			viewer.addPointCloud(clusterClouds[i], color, id + std::to_string(i), v);
			break;
		case 5:
			viewer.addPointCloud(clusterClouds[i], color, id + std::to_string(i), v);
			break;
		case 6:
			viewer.addPointCloud(clusterClouds[i], color, id + std::to_string(i), v);
			break;
		default:
			break;
		}
	}
}

/* =========================================================================== */
/**
* @brief 内積計算(平面パラメータを用いて内積を算出する)
*
* @ param[in]     planeCoeff1  平面パラメータ1
* @ param[in]     planeCoeff2  平面パラメータ2
* @ param[out]    innerPro     内積値
*
*/
/* =========================================================================== */
double CalcuInnerPro(pcl::ModelCoefficients::Ptr planeCoeff1, pcl::ModelCoefficients::Ptr planeCoeff2)
{
	double innerPro;

	innerPro = planeCoeff1->values[0] * planeCoeff2->values[0] +
			   planeCoeff1->values[1] * planeCoeff2->values[1] +
			   planeCoeff1->values[2] * planeCoeff2->values[2];

	return innerPro;
}

/* =========================================================================== */
/**
* @brief 平行投影点郡を2次元座標に変換
*
* @ param[in]      cloud		 変換する点郡
* @ param[out]	   converPoints  変換した2次元座標
*
*/
/* =========================================================================== */
std::vector<pcl::PointXY> convert2d(pcl::PointCloud<PointTypeRGB>::Ptr cloud)
{
	std::vector<pcl::PointXY> convertPoints;
	pcl::PointXY point;

	for (int i = 0; i < cloud->points.size(); i++)
	{
		point.x = cloud->points[i].x;
		point.y = cloud->points[i].z;
		convertPoints.push_back(point);
	}
	return convertPoints;
}

/* =========================================================================== */
/**
* @brief 最小二乗法による円フィッティング
*
* @ param[in]     points	平面に投影した2次元点郡
* @ param[in]     centerX	円の中心のX座標
* @ param[in]     centerY	円の中心のY座標
* @ param[in]	  radius	円の半径
*
*/
/* =========================================================================== */
void fitCircle(std::vector<pcl::PointXY> points, double *centerX, double *centerY, double *radius)
{
	Eigen::MatrixXd x, y, A;
	x = Eigen::MatrixXd::Zero(3, 3);
	y = Eigen::MatrixXd::Zero(3, 1);

	double Xi3 = 0.0;
	double Xi2 = 0.0;
	double Xi = 0.0;
	double Yi3 = 0.0;
	double Yi2 = 0.0;
	double Yi = 0.0;
	double Xi2Yi = 0.0;
	double XiYi2 = 0.0;
	double XiYi = 0.0;

	for (int i = 0; i < points.size(); i++)
	{
		Xi3 += pow(points[i].x, 3);
		Xi2 += pow(points[i].x, 2);
		Xi += points[i].x;
		Yi3 += pow(points[i].y, 3);
		Yi2 += pow(points[i].y, 2);
		Yi += points[i].y;

		Xi2Yi += (pow(points[i].x, 2) * points[i].y);
		XiYi2 += (points[i].x * pow(points[i].y, 2));
		XiYi += (points[i].x * points[i].y);
	}

	x(0, 0) = Xi2;
	x(0, 1) = XiYi;
	x(0, 2) = Xi;
	x(1, 0) = XiYi;
	x(1, 1) = Yi2;
	x(1, 2) = Yi;
	x(2, 0) = Xi;
	x(2, 1) = Yi;
	x(2, 2) = points.size();

	y(0, 0) = -1 * (Xi3 + XiYi2);
	y(1, 0) = -1 * (Xi2Yi + Yi3);
	y(2, 0) = -1 * (Xi2 + Yi2);

	A = x.inverse() * y;
	*centerX = (-1 / 2.0) * A(0);
	*centerY = (-1 / 2.0) * A(1);
	*radius = sqrt(pow(*centerX, 2) + pow(*centerY, 2) - A(2));
}

/* =========================================================================== */
/**
* @ brief 2点間の距離を算出する
* @ param[in]     point1    座標1
* @ param[in]     point2    座標2
* @ param[out]    distance  距離
*/
/* =========================================================================== */
double calcDistance(pcl::PointXYZ point1, pcl::PointXYZ point2)
{
	double distance;
	distance = sqrt(pow((point2.x - point1.x), 2) + pow((point2.y - point1.y), 2) + pow((point2.z - point1.z), 2));
	return distance;
}