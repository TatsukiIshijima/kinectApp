#include "stdafx.h"
#include "PCLPointType.h"
#include "PCLConvexhull.h"

/* =========================================================================== */
/**
* @brief �������e
*
* @ param[in]     inputCloud	���͓_�S
* @ param[in]     xy_cloud		XY���ʓ��e�_�S
* @ param[in]     yz_cloud		YZ���ʓ��e�_�S
* @ param[in]     zx_cloud		ZX���ʓ��e�_�S
*
*/
/* =========================================================================== */
void perspectiveProjection(pcl::PointCloud<PointTypeRGB>::Ptr inputCloud, pcl::PointCloud<PointTypeRGB>::Ptr xy_cloud, pcl::PointCloud<PointTypeRGB>::Ptr yz_cloud, pcl::PointCloud<PointTypeRGB>::Ptr zx_cloud)
{
	for (int i = 0; i < inputCloud->points.size(); i++)
	{
		pcl::PointXYZRGB xy_point, yz_point, zx_point;

		xy_point.x = inputCloud->points[i].x;
		xy_point.y = inputCloud->points[i].y;
		xy_point.z = 0;
		xy_point.r = inputCloud->points[i].r;
		xy_point.g = inputCloud->points[i].g;
		xy_point.b = inputCloud->points[i].b;
		xy_cloud->push_back(xy_point);

		yz_point.x = 0;
		yz_point.y = inputCloud->points[i].y;
		yz_point.z = inputCloud->points[i].z;
		yz_point.r = inputCloud->points[i].r;
		yz_point.g = inputCloud->points[i].g;
		yz_point.b = inputCloud->points[i].b;
		yz_cloud->push_back(yz_point);

		zx_point.x = inputCloud->points[i].x;
		zx_point.y = 0;
		zx_point.z = inputCloud->points[i].z;
		zx_point.r = inputCloud->points[i].r;
		zx_point.g = inputCloud->points[i].g;
		zx_point.b = inputCloud->points[i].b;
		zx_cloud->push_back(zx_point);
	}
}

/* =========================================================================== */
/**
* @brief �ʕ�̎��͒��v�Z
*
* @ param[in]     cloud_vexhull	  �ʕ�_�S
* @ param[out]	  length		  ���͒�
*
*/
/* =========================================================================== */
double calcConvexLength(pcl::PointCloud<PointTypeRGB>::Ptr cloud_vexhull)
{
	double length = 0;

	for (int i = 1; i < cloud_vexhull->points.size(); i++)
	{
		length += sqrt(pow((cloud_vexhull->points[i - 1].x - cloud_vexhull->points[i].x), 2) +
					   pow((cloud_vexhull->points[i - 1].y - cloud_vexhull->points[i].y), 2) +
					   pow((cloud_vexhull->points[i - 1].z - cloud_vexhull->points[i].z), 2));
	}
	length += sqrt(pow((cloud_vexhull->points[cloud_vexhull->points.size() - 1].x - cloud_vexhull->points[0].x), 2) +
				   pow((cloud_vexhull->points[cloud_vexhull->points.size() - 1].y - cloud_vexhull->points[0].y), 2) +
				   pow((cloud_vexhull->points[cloud_vexhull->points.size() - 1].z - cloud_vexhull->points[0].z), 2));

	return length;
}

/* =========================================================================== */
/**
* @brief �ʕ�_�S�A�ʐρA���͒��v�Z
*
* @ param[in]     inputCloud	  ���͓_�S(2����)
* @ param[in]	  cloud_vexhull   �ʕ�_�S
* @ param[in]	  length		  ���͒�
* @ param[in]	  area			  �ʐ�
*
*/
/* =========================================================================== */
void getConvexParam(pcl::PointCloud<PointTypeRGB>::Ptr inputCloud, pcl::PointCloud<PointTypeRGB>::Ptr cloud_vexhull, double *length, double *area)
{
	//pcl::PointCloud<PointTypeRGB>::Ptr cloud_vexhull(new pcl::PointCloud<PointTypeRGB>);
	pcl::ConvexHull<PointTypeRGB> conv;

	conv.setInputCloud(inputCloud);
	conv.setComputeAreaVolume(true);
	conv.reconstruct(*cloud_vexhull);

	*length = calcConvexLength(cloud_vexhull);
	*area = conv.getTotalArea();
}

/* =========================================================================== */
/**
* @brief �ۂݓx���v�Z
*
* @ param[in]     inputCloud	  ���͓_�S(3����)
* @ param[out]	  DoR			  �ۂݓx��
*
*/
/* =========================================================================== */
double calcDoR(pcl::PointCloud<PointTypeRGB>::Ptr inputCloud)
{
	double DoR;
	double xy_length, xy_area;
	double yz_length, yz_area;
	double zx_length, zx_area;

	pcl::PointCloud<PointTypeRGB>::Ptr xy_cloud(new pcl::PointCloud<PointTypeRGB>);
	pcl::PointCloud<PointTypeRGB>::Ptr yz_cloud(new pcl::PointCloud<PointTypeRGB>);
	pcl::PointCloud<PointTypeRGB>::Ptr zx_cloud(new pcl::PointCloud<PointTypeRGB>);

	pcl::PointCloud<PointTypeRGB>::Ptr xy_cloud_vexhull(new pcl::PointCloud<PointTypeRGB>);
	pcl::PointCloud<PointTypeRGB>::Ptr yz_cloud_vexhull(new pcl::PointCloud<PointTypeRGB>);
	pcl::PointCloud<PointTypeRGB>::Ptr zx_cloud_vexhull(new pcl::PointCloud<PointTypeRGB>);

	// �e���ʂɓ��e
	perspectiveProjection(inputCloud, xy_cloud, yz_cloud, zx_cloud);
	// �e���ʂ̓ʕ�̎��͒��A�ʐώZ�o
	getConvexParam(xy_cloud, xy_cloud_vexhull, &xy_length, &xy_area);
	getConvexParam(yz_cloud, yz_cloud_vexhull, &yz_length, &yz_area);
	getConvexParam(zx_cloud, zx_cloud_vexhull, &zx_length, &zx_area);

	double R_xy = (M_PI * pow(xy_length, 2)) / (4 * xy_area);
	double R_yz = (M_PI * pow(yz_length, 2)) / (4 * yz_area);
	double R_zx = (M_PI * pow(zx_length, 2)) / (4 * zx_area);

	double W_sum = xy_area + yz_area + zx_area;
	double W_xy = xy_area / W_sum;
	double W_yz = yz_area / W_sum;
	double W_zx = zx_area / W_sum;

	DoR = R_xy * W_xy + R_yz * W_yz + R_zx * W_zx;

	return DoR;
}