#include "stdafx.h"
#include "PCLPointType.h"
#include "PCLSegmentation.h"

/* =========================================================================== */
/**
* @brief ���ʌ��o�ꏊ�ɐF��t����
*
* @ param[in] cloud      �_�S
*
*/
/* =========================================================================== */
void colored_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	// ���f���p�����[�^�̒萔
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//�@���e�͈͂ɂ�����W�Ȃ�(�C�����C�A)
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZRGB> sac_seg;
	// �Z�O�����e�[�V���������ݒ�
	sac_seg.setOptimizeCoefficients(true);
	// ���f���̐ݒ�
	sac_seg.setModelType(pcl::SACMODEL_PLANE);
	// RANSAC�A���S���Y���̓K��
	sac_seg.setMethodType(pcl::SAC_RANSAC);
	// �J��Ԃ���(�f�t�H���g50)
	sac_seg.setMaxIterations(100);
	// ����臒l
	sac_seg.setDistanceThreshold(0.03);

	// �Z�O�����e�[�V��������_�S�̐ݒ�
	sac_seg.setInputCloud(cloud);
	// �Z�O�����e�[�V�������s
	sac_seg.segment(*inliers, *coefficients);

	pcl::console::print_info("Model coefficients : %d\n", coefficients);

	// ���o����������ԂɕύX(inliers�ɂ͓_�S�̃C���f�b�N�X���i�[����Ă���)
	for (size_t i = 0; i < inliers->indices.size(); i++) {
		cloud->points[inliers->indices[i]].r = 255;
		cloud->points[inliers->indices[i]].g = 0;
		cloud->points[inliers->indices[i]].b = 0;
	}
}

/* =========================================================================== */
/**
* @brief ���ʌ��o
*
* @ param[in]     cloud      �_�S
* @ param[out]   inliners   ���ʓ_�S�̃C���f�b�N�X
*
*/
/* =========================================================================== */
pcl::PointIndices::Ptr planeSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr& cloud)
{
	// ���f���p�����[�^�̒萔
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//�@���e�͈͂ɂ�����W�Ȃ�(�C�����C�A)
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<PointTypeRGB> sac_seg;
	// �Z�O�����e�[�V���������ݒ�
	sac_seg.setOptimizeCoefficients(true);
	// ���f���̐ݒ�
	sac_seg.setModelType(pcl::SACMODEL_PLANE);
	// RANSAC�A���S���Y���̓K��
	sac_seg.setMethodType(pcl::SAC_RANSAC);
	// �J��Ԃ���(�f�t�H���g50)
	sac_seg.setMaxIterations(100);
	// ����臒l
	sac_seg.setDistanceThreshold(0.005);
	// �Z�O�����e�[�V��������_�S�̐ݒ�
	sac_seg.setInputCloud(cloud);
	// �Z�O�����e�[�V�������s
	sac_seg.segment(*inliers, *coefficients);

	std::cerr << "Model coefficients : " << "a : " << coefficients->values[0]
		<< "  b : " << coefficients->values[1]
		<< "  c : " << coefficients->values[2]
		<< "  d : " << coefficients->values[3] << std::endl;

	return inliers;
}

/* =========================================================================== */
/**
* @brief ���ʌ��o
*
* @ param[in]     cloud              ���͓_�S
* @ param[in]     coefficients       ���ʂ̃p�����[�^���i�[����ϐ�
* @ param[in]     inliners           ���ʓ_�S�̃C���f�b�N�X�i�[�p�ϐ�
* @ param[in]     iterations         RANSAC�̌J��Ԃ���
* @ param[in]     distanceThreshold  ������臒l
*
*/
/* =========================================================================== */
void planeSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, int iterations, double distanceThreshold)
{
	// �ꎞ�ۊǂ̓_�S
	pcl::PointCloud<PointTypeRGB>::Ptr tmp(new pcl::PointCloud<PointTypeRGB>);
	// �_�S�R�s�[
	pcl::copyPointCloud(*cloud, *tmp);

	pcl::SACSegmentation<PointTypeRGB> sac_seg;
	// �Z�O�����e�[�V���������ݒ�
	sac_seg.setOptimizeCoefficients(true);
	// ���f���̐ݒ�
	sac_seg.setModelType(pcl::SACMODEL_PLANE);
	// RANSAC�A���S���Y���̓K��
	sac_seg.setMethodType(pcl::SAC_RANSAC);
	// �J��Ԃ���(�f�t�H���g50)
	sac_seg.setMaxIterations(iterations);
	// ����臒l
	sac_seg.setDistanceThreshold(distanceThreshold);
	// �Z�O�����e�[�V��������_�S�̐ݒ�
	sac_seg.setInputCloud(tmp);
	// �Z�O�����e�[�V�������s
	sac_seg.segment(*inliers, *coefficients);
}

/* =========================================================================== */
/**
* @brief �������ʌ��o
*
* @ param[in]     cloud              ���͓_�S
* @ param[in]     iterations         RANSAC�̌J��Ԃ���
* @ param[in]     distanceThreshold  ������臒l
* @ param[in]     k                  ���͓_�S�̐��̒萔�{��菬������΃��[�v���I��������萔
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
		// ���ʌ��o
		planeSegmentation(cloud, coefficients, inliers, iterations, distanceThreshold);

		if (inliers->indices.size() == 0) break;

		// �_�S���o
		extractIndices(inCloud, inliers, false);
		extractIndices(outCloud, inliers, true);

		pcl::PointCloud<PointTypeRGB>::Ptr planePointCloud(new pcl::PointCloud<PointTypeRGB>);
		*planePointCloud = *inCloud;
		planeClouds.push_back(planePointCloud);

		// �_�S�X�V
		*cloud = *outCloud;
	}
	return planeClouds;
}

/* =========================================================================== */
/**
* @brief �~�����o
*
* @ param[in]     cloud     �_�S
* @ param[in]     normal    �@���_�S
* @ param[out]   inliners   �~���_�S�̃C���f�b�N�X
*
*/
/* =========================================================================== */
void cylinderSegmentation(pcl::PointCloud<PointTypeRGB>::Ptr& cloud, pcl::PointCloud<NormalType>::Ptr& normal, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers, double weight, int iterarion, double distance, double min_radius, double max_radius)
{
	// ���f���p�����[�^�̒萔
	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//�@���e�͈͂ɂ�����W�Ȃ�(�C�����C�A)
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentationFromNormals<PointTypeRGB, NormalType> sac_seg_norm;
	// �Z�O�����e�[�V���������ݒ�
	sac_seg_norm.setOptimizeCoefficients(true);
	// ���f���̐ݒ�
	sac_seg_norm.setModelType(pcl::SACMODEL_CYLINDER);
	// RANSAC�A���S���Y���̓K��
	sac_seg_norm.setMethodType(pcl::SAC_RANSAC);
	// (0~1�̊Ԃňȉ���ݒ�)
	sac_seg_norm.setNormalDistanceWeight(weight);
	// �J��Ԃ���
	sac_seg_norm.setMaxIterations(iterarion);
	// ����臒l
	sac_seg_norm.setDistanceThreshold(distance);
	// �~���̍ő唼�a, �ŏ����a
	sac_seg_norm.setRadiusLimits(min_radius, max_radius);
	// �Z�O�����e�[�V��������_�S�̐ݒ�
	sac_seg_norm.setInputCloud(cloud);
	// �@���_�S�̐ݒ�
	sac_seg_norm.setInputNormals(normal);
	// �Z�O�����e�[�V�������s
	sac_seg_norm.segment(*inliers, *coefficients);
}

/* =========================================================================== */
/**
* @brief ���[�N���b�h�N���X�^�����O
*
* @ param[in]     cloud             �_�S
* @ param[in]     normal            �@���_�S
* @ param[in]     tolerance         ���e��(x�Ay�Az���W�n)
* @ param[in]     angle             ���e��(�p�x)
* @ param[in]     min_cluster_size  �e�N���X�^�[�̍ŏ��_�S��
* @ param[out]    cluster_indices   �e�N���X�^�[�̃C���f�b�N�X
*
*/
/* =========================================================================== */
std::vector<pcl::PointIndices> euclideanCluster(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointCloud<NormalType>::Ptr cloud_normal, float tolerance, double angle, int min_cluster_size)
{
	// kdtree�I�u�W�F�N�g�쐬(���o�̒T���@�̂���)
	boost::shared_ptr<pcl::KdTree<PointTypeRGB>> tree_ec(new pcl::KdTreeFLANN<PointTypeRGB>());
	tree_ec->setInputCloud(cloud);

	// �_�S�Ɩ@����p���ă��[�N���b�h�N���X�^�[���o
	std::vector<pcl::PointIndices> cluster_indices;                                       // �N���X�^�[�C���f�b�N�X
	//const float tolerance = 0.02f;                                                        // ���e��(x�Ay�Az���W�n)
	//const double esp_angle = angle * (M_PI / 180.0);                                        // ���e��(�p�x�H)
	double esp_angle = angle * (M_PI / 180.0);
	//const unsigned int min_cluster_size = 500;

	pcl::extractEuclideanClusters(*cloud, *cloud_normal, tolerance, tree_ec, cluster_indices, esp_angle, min_cluster_size);
	pcl::EuclideanClusterExtraction<PointTypeRGB> ec;

	return cluster_indices;
}

/* =========================================================================== */
/**
* @brief ���[�N���b�h�N���X�^�����O���ǔ�
*        �e�N���X�^�[�_�S�ŕԂ��悤�ɉ���
*
* @ param[in]     cloud             �_�S
* @ param[in]     normal            �@���_�S
* @ param[in]     tolerance         ���e��(x�Ay�Az���W�n)
* @ param[in]     angle             ���e��(�p�x)
* @ param[in]     min_cluster_size  �e�N���X�^�[�̍ŏ��_�S��
* @ param[out]    clusterClouds     �e�N���X�^�[�_�S
*
*/
/* =========================================================================== */
std::vector < pcl::PointCloud<PointTypeRGB>::Ptr> newEuclideanCluster(pcl::PointCloud<PointTypeRGB>::Ptr cloud, pcl::PointCloud<NormalType>::Ptr cloud_normal, float tolerance, double angle, int min_cluster_size)
{
	// kdtree�I�u�W�F�N�g�쐬(���o�̒T���@�̂���)
	boost::shared_ptr<pcl::KdTree<PointTypeRGB>> tree_ec(new pcl::KdTreeFLANN<PointTypeRGB>());
	tree_ec->setInputCloud(cloud);

	// �_�S�Ɩ@����p���ă��[�N���b�h�N���X�^�[���o
	std::vector<pcl::PointIndices> cluster_indices;                                       // �N���X�^�[�C���f�b�N�X
	std::vector < pcl::PointCloud<PointTypeRGB>::Ptr > clusterClouds;
	double esp_angle = angle * (M_PI / 180.0);

	pcl::extractEuclideanClusters(*cloud, *cloud_normal, tolerance, tree_ec, cluster_indices, esp_angle, min_cluster_size);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
		pcl::PointCloud<PointTypeRGB>::Ptr cloud_cluster(new pcl::PointCloud<PointTypeRGB>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
			cloud_cluster->points.push_back(cloud->points[*pit]);
		}
		// �e�N���X�^�[�_�S�𓮓I�z��ɕۑ�
		clusterClouds.push_back(cloud_cluster);
	}
	return clusterClouds;
}

/* =========================================================================== */
/**
* @brief ���ʏ���
*
* @ param[in]     cloud				 ���͓_�S
* @ param[in]     iterations         RANSAC�̌J��Ԃ���
* @ param[in]     distanceThreshold  ������臒l
* @ param[in]     k                  ���͓_�S�̐��̒萔�{��菬������΃��[�v���I��������萔
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
* @brief ���ƕ��s�Ȗʂ���������
*
* @ param[in]     cloud					���͓_�S
* @ param[in]     iterations			RANSAC�̌J��Ԃ���
* @ param[in]     distanceThreshold		������臒l
* @ param[in]	  k						���͓_�S�̐��̒萔�{��菬������΃��[�v���I��������萔
* @ param[in]	  floorPlaneCoeff		���̖@���x�N�g��
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
		// ���ʌ��o
		planeSegmentation(copyCloud, planeCoeff, inliers, iterations, distanceThreshold);

		if (inliers->indices.size() == 0) break;

		// ���όv�Z
		double innerPro = planeCoeff->values[0] * floorPlaneCoeff->values[0] +
						  planeCoeff->values[1] * floorPlaneCoeff->values[1] +
						  planeCoeff->values[2] * floorPlaneCoeff->values[2];

		// ���ϒl��臒l����
		if (innerPro >= 0.8 || innerPro <= -0.8) extractIndices(removedCloud, inliers, true);

		// �_�S���o
		extractIndices(outCloud, inliers, true);
		// �_�S�X�V
		*copyCloud = *outCloud;
	}
	return removedCloud;
}