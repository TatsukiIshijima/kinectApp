#include "stdafx.h"
#include "PCLNormal.h"

/* =========================================================================== */
/**
* @brief �@������(�_�S���N���X�^�����O���Ă��̖@�����Z�o����)
*
* @ param[in]     cloud               �_�S
* @ param[out]    cloud_normals  �@���_�S
*
*/
/* =========================================================================== */
pcl::PointCloud<NormalType>::Ptr normalEstimation(pcl::PointCloud<PointTypeRGB>::Ptr cloud)
{
	// �@������N���X���쐬
	//pcl::NormalEstimation<PointType, NormalType> ne;
	pcl::NormalEstimationOMP<PointTypeRGB, NormalType> ne;
	// �_�S�̐ݒ�
	ne.setInputCloud(cloud);

	// �T���A���S���Y����kd-tree��ݒ�
	pcl::search::KdTree<PointTypeRGB>::Ptr tree(new pcl::search::KdTree<PointTypeRGB>());
	ne.setSearchMethod(tree);

	// �@���_�S�̏���
	pcl::PointCloud<NormalType>::Ptr cloud_normals(new pcl::PointCloud<NormalType>);

	// �ߖT�͈͂̐ݒ�
	ne.setRadiusSearch(0.05);
	//ne.setKSearch(50);
	// �@���Z�o
	ne.compute(*cloud_normals);

	return cloud_normals;
}

/* =========================================================================== */
/**
* @brief �@������(�_�S���N���X�^�����O���Ă��̖@�����Z�o����)
*
* @ param[in]     cloud               �_�S
* @ param[out]    cloud_normals  �@���_�S
*
*/
/* =========================================================================== */
pcl::PointCloud<NormalType>::Ptr normalEstimationKinect(pcl::PointCloud<PointTypeRGB>::Ptr cloud)
{
	// �@������N���X���쐬
	//pcl::NormalEstimation<PointType, NormalType> ne;
	//pcl::NormalEstimationOMP<PointTypeRGB, NormalType> ne;
	pcl::IntegralImageNormalEstimation<PointTypeRGB, NormalType> ne;

	// �T���A���S���Y����kd-tree��ݒ�
	//pcl::search::KdTree<PointTypeRGB>::Ptr tree(new pcl::search::KdTree<PointTypeRGB>());
	//ne.setSearchMethod(tree);

	// �ߖT�͈͂̐ݒ�
	//ne.setRadiusSearch(0.05);
	//ne.setKSearch(50);

	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);          // �v�Z���@�̎w��
	ne.setMaxDepthChangeFactor(0.01);                               // �v�Z�Ώۂ̐[���̕ω���臒l
	ne.setNormalSmoothingSize(5.0);                                 // �@���̕������̃T�C�Y

	// �@���_�S�̏���
	pcl::PointCloud<NormalType>::Ptr cloud_normals(new pcl::PointCloud<NormalType>);

	// �_�S�̐ݒ�
	ne.setInputCloud(cloud);
	// �@���Z�o
	ne.compute(*cloud_normals);

	return cloud_normals;
}
