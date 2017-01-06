#include "stdafx.h"
#include "PCLPointType.h"
#include "PCLFilter.h"

/* =========================================================================== */
/**
* @brief ���͈͓��̐��Ńt�B���^�[����
*
* @ param[in] cloud   �_�S
* @ param[in] leaf    �t�B���^�[����͈�
*
*/
/* =========================================================================== */
void voxelGridFilter(pcl::PointCloud<PointTypeRGB>::Ptr cloud, float leaf)
{
	// �t�B���^�[�O�̓_�S�̐���\������
	//pcl::console::print_info("before VoxelGridFilter point clouds : %d\n", cloud->size());

	pcl::VoxelGrid<PointTypeRGB> grid;

	// �t�B���^�[����͈͂�ݒ�
	grid.setLeafSize(leaf, leaf, leaf);

	// �t�B���^�[����_�S��ݒ�
	grid.setInputCloud(cloud);

	// �t�B���^�[��̓_�S��ۑ����邽�߂̃N���E�h
	pcl::PointCloud<PointTypeRGB>::Ptr filtered_cloud(new pcl::PointCloud<PointTypeRGB>);
	// �O���b�h�t�B���^�[���s
	grid.filter(*filtered_cloud);

	// �t�B���^�[��̓_�S���R�s�[
	pcl::copyPointCloud(*filtered_cloud, *cloud);

	// �t�B���^�[��̓_�S�̐���\������
	//pcl::console::print_info("after VoxelGridFilter point clouds : %d\n", cloud->size());
}

/* =========================================================================== */
/**
* @brief ���͈͓��̐��Ńt�B���^�[����
*
* @ param[in] cloud   �_�S
* @ param[in] leaf    �t�B���^�[����͈�
*
*/
/* =========================================================================== */
void voxelGridFilterRGBNormal(pcl::PointCloud<PointTypeRGBNormal>::Ptr cloud_color_normal, float leaf)
{
	// �t�B���^�[�O�̓_�S�̐���\������
	//pcl::console::print_info("before VoxelGridFilter point clouds : %d\n", cloud->size());

	pcl::VoxelGrid<PointTypeRGBNormal> grid;

	// �t�B���^�[����͈͂�ݒ�
	grid.setLeafSize(leaf, leaf, leaf);

	// �t�B���^�[����_�S��ݒ�
	grid.setInputCloud(cloud_color_normal);

	// �t�B���^�[��̓_�S��ۑ����邽�߂̃N���E�h
	pcl::PointCloud<PointTypeRGBNormal>::Ptr filtered_cloud(new pcl::PointCloud<PointTypeRGBNormal>);
	// �O���b�h�t�B���^�[���s
	grid.filter(*filtered_cloud);

	// �t�B���^�[��̓_�S���R�s�[
	pcl::copyPointCloud(*filtered_cloud, *cloud_color_normal);

	// �t�B���^�[��̓_�S�̐���\������
	//pcl::console::print_info("after VoxelGridFilter point clouds : %d\n", cloud->size());
}

/* =========================================================================== */
/**
* @brief �w�肵������(��)�ɂ��āC�l�͈̔͂Ńt�B���^�[����
*
* @ param[in] cloud      �_�S
* @ param[in] axis       ����(��)
* @ param[in] limit_min  �͈͂̏���l
* @ param[in] limit_max  �͈͂̉����l
* @ param[in] keep       organized�_�S�Ƃ��ăL�[�v���邩
*
*/
/* =========================================================================== */
void passThroughFilter(pcl::PointCloud<PointTypeRGB>::Ptr cloud, const std::string &axis, float limit_min, float limit_max, bool keep)
{
	// �t�B���^�[�O�̓_�S�̐���\������
	//pcl::console::print_info("before PassThroughFilter point clouds : %d\n", cloud->size());

	pcl::PassThrough<PointTypeRGB> pass;

	// �t�B���^�[����_�S��ݒ�
	pass.setInputCloud(cloud);
	pass.setKeepOrganized(keep);
	// �t�B���^�[���鎟��(��)�̐ݒ�
	pass.setFilterFieldName(axis);
	// �t�B���^�[����͈͂̐ݒ�
	pass.setFilterLimits(limit_min, limit_max);

	// �t�B���^�[��̓_�S��ۑ����邽�߂̃N���E�h
	pcl::PointCloud<PointTypeRGB>::Ptr filtered_cloud(new pcl::PointCloud<PointTypeRGB>);
	// �O���b�h�t�B���^�[���s
	pass.filter(*filtered_cloud);

	// �t�B���^�[��̓_�S���R�s�[
	pcl::copyPointCloud(*filtered_cloud, *cloud);

	// �t�B���^�[��̓_�S�̐���\������
	//pcl::console::print_info("after PassThroughFilter point clouds : %d\n", cloud->size());
}

/* =========================================================================== */
/**
* @brief �w�肵������(��)�ɂ��āC�l�͈̔͂Ńt�B���^�[����
*
* @ param[in] cloud      �_�S
* @ param[in] axis       ����(��)
* @ param[in] limit_min  �͈͂̏���l
* @ param[in] limit_max  �͈͂̉����l
* @ param[in] keep       organized�_�S�Ƃ��ăL�[�v���邩
*
*/
/* =========================================================================== */
void passThroughFilterRGBNormal(pcl::PointCloud<PointTypeRGBNormal>::Ptr cloud_color_normal, const std::string &axis, float limit_min, float limit_max, bool keep)
{
	// �t�B���^�[�O�̓_�S�̐���\������
	//pcl::console::print_info("before PassThroughFilter point clouds : %d\n", cloud->size());

	pcl::PassThrough<PointTypeRGBNormal> pass;

	// �t�B���^�[����_�S��ݒ�
	pass.setInputCloud(cloud_color_normal);
	pass.setKeepOrganized(keep);
	// �t�B���^�[���鎟��(��)�̐ݒ�
	pass.setFilterFieldName(axis);
	// �t�B���^�[����͈͂̐ݒ�
	pass.setFilterLimits(limit_min, limit_max);

	// �t�B���^�[��̓_�S��ۑ����邽�߂̃N���E�h
	pcl::PointCloud<PointTypeRGBNormal>::Ptr filtered_cloud(new pcl::PointCloud<PointTypeRGBNormal>);
	// �O���b�h�t�B���^�[���s
	pass.filter(*filtered_cloud);

	// �t�B���^�[��̓_�S���R�s�[
	pcl::copyPointCloud(*filtered_cloud, *cloud_color_normal);

	// �t�B���^�[��̓_�S�̐���\������
	//pcl::console::print_info("after PassThroughFilter point clouds : %d\n", cloud->size());
}

/* =========================================================================== */
/**
* @brief �m�C�Y�����t�B���^�[
*
* @ param[in] cloud      �_�S
* @ param[in] n          �ߖT�_��
* @ param[in] sdm        �W���΍��搔(standard deviation multiplier)
*
*/
/* =========================================================================== */
void removalFilter(pcl::PointCloud<PointTypeRGB>::Ptr cloud, int n, float sdm)
{
	pcl::StatisticalOutlierRemoval<PointTypeRGB> sor;
	// �t�B���^�[����_�S��ݒ�
	sor.setInputCloud(cloud);
	// �ߖT�_���̐ݒ�
	sor.setMeanK(n);
	// �W���΍��搔�̐ݒ�
	sor.setStddevMulThresh(sdm);
	// �t�B���^�[��̓_�S��ۑ����邽�߂̃N���E�h
	pcl::PointCloud<PointTypeRGB>::Ptr filtered_cloud(new pcl::PointCloud<PointTypeRGB>);
	// �t�B���^�[�̎��s
	sor.filter(*filtered_cloud);
	// �t�B���^�[��̓_�S���R�s�[
	pcl::copyPointCloud(*filtered_cloud, *cloud);
}

/* =========================================================================== */
/**
* @brief ���͓_�S����C�����C�A���̂��̂܂��̓C�����C�A�O�̓_�S�𒊏o����������
*
* @ param[in] cloud      �_�S
* @ param[in] inliers     �C�����C�A�_�S(���ʂ�~�������o���ꂽ�_�S)
* @ param[in] negative �C�����C�A��(true)���C�����C�A�O(false)
*/
/* =========================================================================== */
void extractIndices(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative)
{
	// �ꎞ�ۊǂ̓_�S
	pcl::PointCloud<PointTypeRGB>::Ptr tmp(new pcl::PointCloud<PointTypeRGB>);
	// �_�S�R�s�[
	pcl::copyPointCloud(*cloud, *tmp);

	// �_�S�̒��o
	pcl::ExtractIndices<PointTypeRGB> extract;
	// ���͓_�S�̐ݒ�
	extract.setInputCloud(tmp);
	// ���͓_�S���璊�o����_�S�̐ݒ�(�C�����C�A)
	extract.setIndices(inliers);

	// true�ɂ����inliers���̓_�S����,false�ɂ����inliers�ȊO������
	extract.setNegative(negative);
	// �t�B���^�̎��s
	extract.filter(*cloud);
}

/* =========================================================================== */
/**
* @brief ���͖@���_�S����C�����C�A���̂��̂܂��̓C�����C�A�O�̖@���_�S�𒊏o����������
*
* @ param[in] normal     �@���_�S
* @ param[in] inliers       �C�����C�A�_�S(���ʂ�~�������o���ꂽ�_�S)
* @ param[in] negative   �C�����C�A��(true)���C�����C�A�O(false)
*/
/* =========================================================================== */
void extractIndiciesNormal(pcl::PointCloud<NormalType>::Ptr normal, pcl::PointIndices::Ptr inliers, bool negative)
{
	// �ꎞ�ۊǂ̖@���_�S
	pcl::PointCloud<NormalType>::Ptr tmp(new pcl::PointCloud<NormalType>);
	// �@���_�S�R�s�[
	pcl::copyPointCloud(*normal, *tmp);

	// �@���̒��o
	pcl::ExtractIndices<NormalType> extract_normal;
	// ���͖@���̐ݒ�
	extract_normal.setInputCloud(tmp);
	// ���͖@�����璊�o����@���_�S�̐ݒ�(�C�����C�A)
	extract_normal.setIndices(inliers);

	// true�ɂ����inliers���̖@������,false �ɂ����inliers�ȊO�̖@������
	extract_normal.setNegative(negative);
	// ���o���s
	extract_normal.filter(*normal);
}