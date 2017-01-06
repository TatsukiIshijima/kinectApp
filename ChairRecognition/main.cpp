// ChairRecognition.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include "stdafx.h"
#include <sys/stat.h>
// C��min,max�}�N���𖳌��ɂ���
#define NOMINMAX
#define USE_GESTURE

// ���S�łȂ����\�b�h�̌Ăяo���ł̌x���𖳌��ɂ���
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "PCLFilter.h"
#include "PCLMomentOfInertia.h"
#include "PCLNormal.h"
#include "PCLSearch.h"
#include "PCLSegmentation.h"
#include "kinect2_grabber.h"
#include "PCLNtkinect.h"

#include <pcl\registration\transforms.h>

NtKinect kinect;

/* =========================================================================== */
/**
* @brief �ł��ڐG�_���������_�S�̃C���f�b�N�X���擾����
*
* @ param[in]     clusterClouds  �_�S
* @ param[in]     searchPoint    �T����_
* @ param[in]     resolution     �{�N�Z�������H
* @ param[in]     redius         ���a
* @ param[out]    index          �N���X�^�[�C���f�b�N�X
*
*/
/* =========================================================================== */
int getTouchedClsterIndex(std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> clusterClouds, pcl::PointXYZRGB searchPoint, float resolution, float radius)
{
	pcl::PointCloud<PointTypeRGB>::Ptr bodyCloud(new pcl::PointCloud<PointTypeRGB>);
	std::vector<int> pointArray;
	int index;

	for (int i = 0; i < clusterClouds.size(); i++)
	{
		bodyCloud = RadiusSearch(clusterClouds[i], resolution, searchPoint, radius);
		pointArray.push_back(bodyCloud->points.size());
	}

	int sum = accumulate(pointArray.begin(), pointArray.end(), 0);

	if (sum != 0)
	{
		std::vector<int>::iterator iter = std::max_element(pointArray.begin(), pointArray.end());
		index = std::distance(pointArray.begin(), iter);
		return index;
	}
	else return NULL;
}

/* =========================================================================== */
/**
* @brief �����̕��ʓ_�S����d�S��Y���W���ŏ��ȕ��ʓ_�S�Ƃ��̏d�S���擾
*
* @ param[in]     planeClouds    ���ʓ_�S�z��
* @ param[in]     floorCloud     ���ʓ_�S
* @ param[in]	  centroid		 �d�S
*
*/
/* =========================================================================== */
void detectYminPlane(std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> planeClouds, pcl::PointCloud<PointTypeRGB>::Ptr floorCloud, Eigen::Vector4f* minCentroid)
{
	Eigen::Vector4f centroid;
	double min = 100;
	
	for (int i = 0; i < planeClouds.size(); i++)
	{
		pcl::compute3DCentroid(*planeClouds[i], centroid);
		if (min > centroid[1])
		{
			min = centroid[1];
			pcl::copyPointCloud(*planeClouds[i], *floorCloud);
			*minCentroid = centroid;
		}
	}
}

/* =========================================================================== */
/**
* @ brief �Z���g���C�h���|�C���g�N���E�h�ɕϊ�
* @ param[in]	  centroid	�d�S
* @ param[out]    point		�Z���g���C�h���W(PCL�`��)
*/
/* =========================================================================== */
pcl::PointXYZRGB centroidToPoint(Eigen::Vector4f centroid)
{
	pcl::PointXYZRGB point;

	point.x = centroid[0];
	point.y = centroid[1];
	point.z = centroid[2];

	return point;
}

int main()
{
	try
	{
		/* =========================================================================== */
		/*                                 �ϐ��錾                                    */
		/* =========================================================================== */
		pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

		int v1(0);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.setBackgroundColor(0, 0, 0, v1);

		int v2(0);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(0.2, 0.2, 0.2, v2);

		viewer.addCoordinateSystem(1.0);

		pcl::PointCloud<PointTypeRGB>::Ptr cloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr proCloud(new pcl::PointCloud<PointTypeRGB>);		// �����p
		pcl::PointCloud<PointTypeRGB>::Ptr tmpCloud(new pcl::PointCloud<PointTypeRGB>);		// �ꎞ�ۑ��p
		pcl::PointCloud<PointTypeRGB>::Ptr floorCloud(new pcl::PointCloud<PointTypeRGB>);	// ���_�S
		pcl::PointCloud<PointTypeRGB>::Ptr chairCloud(new pcl::PointCloud<PointTypeRGB>);	// �C�X�p
		pcl::PointCloud<PointTypeRGB>::Ptr inCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr outCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr footTouchCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGBNormal>::Ptr RGBNormalCloud(new pcl::PointCloud<PointTypeRGBNormal>);
		pcl::PointCloud<NormalType>::Ptr normalCloud(new pcl::PointCloud<NormalType>);

		pcl::ModelCoefficients::Ptr floorCoeff(new pcl::ModelCoefficients);					// ���̖@���x�N�g��
		//pcl::ModelCoefficients::Ptr kinetFloorCoeff(new pcl::ModelCoefficients);

		std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> clusterClouds;						// �N���X�^�[�_�S�z��
		std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> planeClouds;						// ���ʓ_�S�z��
		std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> parallelPlaneClouds;				// ���s���ʓ_�S�z��

		bool floorDetect = false;															// �����o�t���O
		Eigen::Vector4f centroid;															// ���̏d�S

		std::vector<pcl::PointXYZRGB> planeCentroids;										// ���s���ʂ̏d�S�z��
		pcl::PointXYZRGB sitBasePoint, sitHistPoint;
		double sitScore;
		int count = 1;

		/* =========================================================================== */
		/*                               kinect������                                  */
		/* =========================================================================== */
		kinect.setGestureFile(L"SitandStand.gbd");
		boost::mutex mutex;
		boost::function<void(const pcl::PointCloud<PointTypeRGB>::ConstPtr&)>
			function = [&cloud, &mutex](const pcl::PointCloud<PointTypeRGB>::ConstPtr &newCloud)
		{
			boost::mutex::scoped_lock lock(mutex);
			pcl::copyPointCloud(*newCloud, *cloud);
		};
		// Kinect2Grabber�̊J�n
		pcl::Kinect2Grabber grabber;
		grabber.registerCallback(function);
		grabber.start();
		// �o�͂����肷��܂ł̑ҋ@����
		Sleep(5000);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			viewer.removeAllPointClouds();
			viewer.removeAllShapes();

			/* =========================================================================== */
			/*                             ���i���o&����F��                               */
			/* =========================================================================== */
			pcl::PointXYZRGB leftFoot, rightFoot, footPoint, basePoint;

			kinect.setSkeleton();
			for (auto person : kinect.skeleton)
			{
				for (auto joint : person)
				{
					if (joint.TrackingState == TrackingState_NotTracked) continue;
					// ����
					if (joint.JointType == JointType_FootLeft)
					{
						leftFoot.x = joint.Position.X;
						leftFoot.y = joint.Position.Y;
						leftFoot.z = joint.Position.Z;
					}
					// �E��
					if (joint.JointType == JointType_FootRight)
					{
						rightFoot.x = joint.Position.X;
						rightFoot.y = joint.Position.Y;
						rightFoot.z = joint.Position.Z;

					}
					// �����ƉE���̒���
					if (leftFoot.z != 0 && rightFoot.z != 0)
					{
						footPoint.x = (leftFoot.x + rightFoot.x) / 2;
						footPoint.y = (leftFoot.y + rightFoot.y) / 2;
						footPoint.z = (leftFoot.z + rightFoot.z) / 2;
					}
					// ��
					if (joint.JointType == JointType_SpineBase)
					{
						basePoint.x = joint.Position.X;
						basePoint.y = joint.Position.Y;
						basePoint.z = joint.Position.Z;
					}
				}
			}

			kinect.setGesture();
			for (int i = 0; i < kinect.discreteGesture.size(); i++)
			{
				auto g = kinect.discreteGesture[i];

				if (kinect.gesture2string(g.first) == "sit" && basePoint.z != 0)
				{
					sitBasePoint.x = basePoint.x;
					sitBasePoint.y = basePoint.y;
					sitBasePoint.z = basePoint.z;
					sitScore = g.second;
					viewer.removeShape("Sit Score", v1);
					viewer.addText("Sit : " + std::to_string(g.second), 10, 90, "Sit Score", v1);
				}
			}

			/* =========================================================================== */
			/*                                   �_�S����                                  */
			/* =========================================================================== */
			boost::mutex::scoped_try_lock lock(mutex);
			if ((cloud->size() != 0) && lock.owns_lock())
			{
				// �@������
				pcl::copyPointCloud(*cloud, *proCloud);
				passThroughFilter(cloud, "z", 0.00, 4.50, false);
				pcl::copyPointCloud(*cloud, *chairCloud);
				passThroughFilter(proCloud, "z", 0.00, 4.50, true);
				normalCloud = normalEstimationKinect(proCloud);
				pcl::concatenateFields(*proCloud, *normalCloud, *RGBNormalCloud);
				voxelGridFilterRGBNormal(RGBNormalCloud, 0.015f);
				pcl::copyPointCloud(*RGBNormalCloud, *proCloud);
				//pcl::copyPointCloud(*RGBNormalCloud, *normalCloud);
				//viewer.addPointCloud(proCloud, "cloud", v1);
				
				// ���̓���(�@���x�N�g��)
				if (floorDetect == false)
				{
					// �������ʌ��o
					planeClouds = planesSegmentation(proCloud, 50, 0.035, 0.4);
					//drawClusterPointCloud(viewer, planeClouds, "planes", v1);
					// �������ŏ��̕��ʂƂ��̏d�S���o
					detectYminPlane(planeClouds, floorCloud, &centroid);
					// �ڐG����
					footTouchCloud = RadiusSearch(floorCloud, 128.0f, footPoint, 0.075f);
					//viewer.addText("Touch Cloud Points : " + std::to_string(footTouchCloud->points.size()), 10, 30, "touch points", v1);
					if (footTouchCloud->points.size() != 0)
					{
						// ���̖@���x�N�g���擾
						pcl::PointIndices::Ptr floorInliers(new pcl::PointIndices);
						planeSegmentation(floorCloud, floorCoeff, floorInliers, 50, 0.035);
						floorDetect = true;
					}
					planeClouds.clear();
				}
				
				else
				{
					std::string k_param = "X : " + std::to_string(kinect.floor.x) +
									   " , Y : " + std::to_string(kinect.floor.y) +
									   " , Z : " + std::to_string(kinect.floor.z) +
									   " , W : " + std::to_string(kinect.floor.w);
					std::string param = "X : " + std::to_string(-1 * floorCoeff->values[0]) +
									 " , Y : " + std::to_string(-1 * floorCoeff->values[1]) +
									 " , Z : " + std::to_string(-1 * floorCoeff->values[2]);
					//viewer.addText("Kinect Floor normal param " + k_param, 10, 10, "Kinect Floor Param", v1);
					viewer.addText("Floor normal param " + param, 10, 10, "Floor Param", v1);

					// ������ɕ␳
					Eigen::Vector3f floor(-1 * floorCoeff->values[0], -1 * floorCoeff->values[1], -1 * floorCoeff->values[2]);
					Eigen::Quaternionf rotate = Eigen::Quaternionf::FromTwoVectors(floor, Eigen::Vector3f::UnitY());
					Eigen::DiagonalMatrix<float, 3> scaling = Eigen::Scaling(1.0f, 1.0f, 1.0f);
					//Eigen::Translation<float, 3> translation(0, kinect.floor.w, 0);
					//Eigen::Affine3f floorMatrix = translation * scaling * rotate;
					//pcl::transformPointCloud(*cloud, *cloud, floorMatrix);
					//viewer.addPointCloud(cloud, "transformed cloud", v1);

					// kinect�̍����̌v�Z
					Eigen::Translation<float, 3> trans1(0, 0, 0);
					Eigen::Affine3f AffineMatrix1 = trans1 * scaling * rotate;
					pcl::PointXYZRGB centroidPoint;
					centroidPoint = centroidToPoint(centroid);
					pcl::PointCloud<PointTypeRGB>::Ptr transCloud(new pcl::PointCloud<PointTypeRGB>);
					transCloud->push_back(centroidPoint);
					//viewer.addText("Centroid Y " + std::to_string(-1 * transCloud->points[0].y), 10, 10, "centroid y", v1);
					pcl::transformPointCloud(*transCloud,  *transCloud, AffineMatrix1);
					//viewer.addText("After Centroid Y " + std::to_string(-1 * transCloud->points[0].y), 10, 30, "after centroid y", v1);
					
					// �␳
					Eigen::Translation<float, 3> trans2(0, -1 * transCloud->points[0].y, 0);
					Eigen::Affine3f AffineMatrix2 = trans2 * scaling * rotate;
					pcl::transformPointCloud(*chairCloud, *chairCloud, AffineMatrix2);
					pcl::transformPointCloudWithNormals(*RGBNormalCloud, *RGBNormalCloud, AffineMatrix2);
					pcl::copyPointCloud(*RGBNormalCloud, *tmpCloud);
					pcl::copyPointCloud(*RGBNormalCloud, *normalCloud);
					
					// �N���X�^�����O
					clusterClouds = newEuclideanCluster(tmpCloud, normalCloud, 0.02f, 100, 500);
					//drawClusterPointCloud(viewer, clusterClouds, "cluster", v1);
					viewer.addText("Cluster Numbers : " + std::to_string(clusterClouds.size()), 10, 30, "cluster num", v1);

					// ���ʌ��o
					for (int i = 0; i < clusterClouds.size(); i++)
					{
						if (clusterClouds[i]->points.size() < 500) continue;
						int nr_points = (int)clusterClouds[i]->points.size();
						while (clusterClouds[i]->points.size() > 0.4 * nr_points)
						{
							pcl::copyPointCloud(*clusterClouds[i], *inCloud);
							pcl::copyPointCloud(*clusterClouds[i], *outCloud);

							pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
							pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

							planeSegmentation(clusterClouds[i], coefficients, inliers, 50, 0.035);

							if (inliers->indices.size() == 0) break;

							// ���ƕ��s������
							double innerPro = CalcuInnerPro(floorCoeff, coefficients);
							if (innerPro >= 0.8 || innerPro <= -0.8)
							{
								// ���s���ʒ��o
								extractIndices(inCloud, inliers, false);
								pcl::PointCloud<PointTypeRGB>::Ptr planePointCloud(new pcl::PointCloud<PointTypeRGB>);
								*planePointCloud = *inCloud;
								parallelPlaneClouds.push_back(planePointCloud);
								// �d�S�Z�o
								Eigen::Vector4f p_planeCentroid;
								pcl::PointXYZRGB p_planeCentroidPoint;
								pcl::compute3DCentroid(*planePointCloud, p_planeCentroid);
								p_planeCentroidPoint = centroidToPoint(p_planeCentroid);
								planeCentroids.push_back(p_planeCentroidPoint);
							}
							extractIndices(outCloud, inliers, true);
							*clusterClouds[i] = *outCloud;
						}
					}
					viewer.addPointCloud(tmpCloud, "tmp cloud", v1);
					viewer.addText("Plane Numbers : " + std::to_string(parallelPlaneClouds.size()), 10, 50, "plane num", v1);
					//drawClusterPointCloud(viewer, parallelPlaneClouds, "parallelPlaneCloud", v1);
					
					if (sitBasePoint.z != 0 && sitScore >= 0.50)
					{
						// ���������̍��̍��W���L�^
						pcl::PointCloud<PointTypeRGB>::Ptr baseTransCloud(new pcl::PointCloud<PointTypeRGB>);
						baseTransCloud->push_back(sitBasePoint);
						pcl::transformPointCloud(*baseTransCloud, *baseTransCloud, AffineMatrix2);
						sitHistPoint = baseTransCloud->points[0];
					}
					viewer.addSphere(sitHistPoint, 0.02, 0, 255, 255, "basePoint", v1);
					viewer.addText("Sit Point X : " + std::to_string(sitHistPoint.x) + " Y : " + std::to_string(sitHistPoint.y) + " Z : " + std::to_string(sitHistPoint.z), 10, 70, "sitBasePoint", v1);

					// ���̍��W�ƐڐG���Ă��镽�ʂ𒊏o
					int index = getTouchedClsterIndex(parallelPlaneClouds, sitHistPoint, 128.0f, 0.30);

					if (index != NULL)
					{
						pcl::visualization::PointCloudColorHandlerCustom<PointTypeRGB> red(parallelPlaneClouds[index], 255, 0, 0);
						viewer.addPointCloud(parallelPlaneClouds[index], red, "extractedPlaneCloud", v1);
						double height = planeCentroids[index].y;
						pcl::PointXYZRGB min_pointAABB;
						pcl::PointXYZRGB max_pointAABB;
						MomentOfInertiaByAABB(parallelPlaneClouds[index], &min_pointAABB, &max_pointAABB);
						viewer.addCube(min_pointAABB.x - 0.10, max_pointAABB.x + 0.10, min_pointAABB.y - height + 0.10, max_pointAABB.y + height + 0.10, min_pointAABB.z - 0.15, max_pointAABB.z + 0.15, 1.0, 1.0, 1.0, "OBB", v1);

						// �C�X�̈�̒��o
						passThroughFilter(chairCloud, "x", min_pointAABB.x - 0.10, max_pointAABB.x + 0.10, false);
						passThroughFilter(chairCloud, "y", min_pointAABB.y - height + 0.10, max_pointAABB.y + height + 0.10, false);
						passThroughFilter(chairCloud, "z", min_pointAABB.z - 0.15, max_pointAABB.z + 0.15, false);
						viewer.addPointCloud(chairCloud, "chair", v2);
						//pcl::io::savePCDFileBinary("chair" + std::to_string(count) + ".pcd", *chairCloud);
						count++;
					}
					parallelPlaneClouds.clear();
					planeCentroids.clear();
				}
			}

			if (GetKeyState(VK_ESCAPE) < 0)
			{
				break;
			}
		}
		grabber.stop();
	}
	catch (const std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
	}
    return 0;
}

