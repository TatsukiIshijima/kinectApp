#include "stdafx.h"
#include "PCLPointType.h"
#include <iostream>
#include <sstream>
#include <Windows.h>
// �ȉ��g���̂����C���N���[�h
#include <pcl\io\pcd_io.h>
#include <pcl\io\ply_io.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl\console\print.h>

//�e�_�S�ɐF��t����
void drawClusterPointCloud(pcl::visualization::PCLVisualizer& viewer, std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> clusterClouds, std::string id, int v);
// ���όv�Z
double CalcuInnerPro(pcl::ModelCoefficients::Ptr planeCoeff1, pcl::ModelCoefficients::Ptr plnaeCoeff2);
// ���s���e�_�S��2�������W�ϊ�
std::vector<pcl::PointXY> convert2d(pcl::PointCloud<PointTypeRGB>::Ptr cloud);
// �ŏ����@�ɂ��~�t�B�b�e�B���O
void fitCircle(std::vector<pcl::PointXY> points, double *centerX, double *centerY, double *radius);
// 2�_�Ԃ̋������Z�o����
double calcDistance(pcl::PointXYZ point1, pcl::PointXYZ point2);