#include "stdafx.h"
// Cのmin,maxマクロを無効にする
#define NOMINMAX
// エラーを無視する
#pragma warning(disable:4996)

#include <pcl\io\io.h>

// 点郡の型を定義
typedef pcl::PointXYZ PointType;                                                                      // XYZ座標
typedef pcl::PointXYZRGB PointTypeRGB;                                                                // XYZ座標 + 色
typedef pcl::Normal NormalType;                                                                       // 法線
typedef pcl::Label LabelType;
typedef pcl::PointXYZRGBNormal PointTypeRGBNormal;
