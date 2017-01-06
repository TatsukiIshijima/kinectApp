#include "stdafx.h"
#include "PCLPointType.h"
#include <iostream>
#include <sstream>
#include <Windows.h>
// 以下使うのもをインクルード
#include <pcl\io\pcd_io.h>
#include <pcl\io\ply_io.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl\console\print.h>

// 内積計算
double CalcuInnerPro(pcl::ModelCoefficients::Ptr planeCoeff1, pcl::ModelCoefficients::Ptr plnaeCoeff2);
// 平行投影点郡を2次元座標変換
std::vector<pcl::PointXY> convert2d(pcl::PointCloud<PointTypeRGB>::Ptr cloud);
// 最小二乗法による円フィッティング
void fitCircle(std::vector<pcl::PointXY> points, double *centerX, double *centerY, double *radius);