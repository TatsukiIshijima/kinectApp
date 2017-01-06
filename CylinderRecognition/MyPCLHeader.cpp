#include "stdafx.h"
#include "MyPCLHeader.h"

/* =========================================================================== */
/**
* @brief ���όv�Z(���ʃp�����[�^��p���ē��ς��Z�o����)
*
* @ param[in]     planeCoeff1  ���ʃp�����[�^1
* @ param[in]     planeCoeff2  ���ʃp�����[�^2
* @ param[out]    innerPro     ���ϒl
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
* @brief ���s���e�_�S��2�������W�ɕϊ�
*
* @ param[in]      cloud		 �ϊ�����_�S
* @ param[out]	   converPoints  �ϊ�����2�������W
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
* @brief �ŏ����@�ɂ��~�t�B�b�e�B���O
*
* @ param[in]     points	���ʂɓ��e����2�����_�S
* @ param[in]     centerX	�~�̒��S��X���W
* @ param[in]     centerY	�~�̒��S��Y���W
* @ param[in]	  radius	�~�̔��a
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