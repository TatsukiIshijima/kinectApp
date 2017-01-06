// ChairRecognition.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "stdafx.h"
#include <sys/stat.h>
// Cのmin,maxマクロを無効にする
#define NOMINMAX
#define USE_GESTURE

// 安全でないメソッドの呼び出しでの警告を無効にする
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
* @brief 最も接触点数が多い点郡のインデックスを取得する
*
* @ param[in]     clusterClouds  点郡
* @ param[in]     searchPoint    探索基準点
* @ param[in]     resolution     ボクセル長さ？
* @ param[in]     redius         半径
* @ param[out]    index          クラスターインデックス
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
* @brief 複数の平面点郡から重心のY座標が最小な平面点郡とその重心を取得
*
* @ param[in]     planeClouds    平面点郡配列
* @ param[in]     floorCloud     平面点郡
* @ param[in]	  centroid		 重心
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
* @ brief セントロイドをポイントクラウドに変換
* @ param[in]	  centroid	重心
* @ param[out]    point		セントロイド座標(PCL形式)
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
		/*                                 変数宣言                                    */
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
		pcl::PointCloud<PointTypeRGB>::Ptr proCloud(new pcl::PointCloud<PointTypeRGB>);		// 処理用
		pcl::PointCloud<PointTypeRGB>::Ptr tmpCloud(new pcl::PointCloud<PointTypeRGB>);		// 一時保存用
		pcl::PointCloud<PointTypeRGB>::Ptr floorCloud(new pcl::PointCloud<PointTypeRGB>);	// 床点郡
		pcl::PointCloud<PointTypeRGB>::Ptr chairCloud(new pcl::PointCloud<PointTypeRGB>);	// イス用
		pcl::PointCloud<PointTypeRGB>::Ptr inCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr outCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr footTouchCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGBNormal>::Ptr RGBNormalCloud(new pcl::PointCloud<PointTypeRGBNormal>);
		pcl::PointCloud<NormalType>::Ptr normalCloud(new pcl::PointCloud<NormalType>);

		pcl::ModelCoefficients::Ptr floorCoeff(new pcl::ModelCoefficients);					// 床の法線ベクトル
		//pcl::ModelCoefficients::Ptr kinetFloorCoeff(new pcl::ModelCoefficients);

		std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> clusterClouds;						// クラスター点郡配列
		std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> planeClouds;						// 平面点郡配列
		std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> parallelPlaneClouds;				// 平行平面点郡配列

		bool floorDetect = false;															// 床検出フラグ
		Eigen::Vector4f centroid;															// 床の重心

		std::vector<pcl::PointXYZRGB> planeCentroids;										// 平行平面の重心配列
		pcl::PointXYZRGB sitBasePoint, sitHistPoint;
		double sitScore;
		int count = 1;

		/* =========================================================================== */
		/*                               kinect準備等                                  */
		/* =========================================================================== */
		kinect.setGestureFile(L"SitandStand.gbd");
		boost::mutex mutex;
		boost::function<void(const pcl::PointCloud<PointTypeRGB>::ConstPtr&)>
			function = [&cloud, &mutex](const pcl::PointCloud<PointTypeRGB>::ConstPtr &newCloud)
		{
			boost::mutex::scoped_lock lock(mutex);
			pcl::copyPointCloud(*newCloud, *cloud);
		};
		// Kinect2Grabberの開始
		pcl::Kinect2Grabber grabber;
		grabber.registerCallback(function);
		grabber.start();
		// 出力が安定するまでの待機時間
		Sleep(5000);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			viewer.removeAllPointClouds();
			viewer.removeAllShapes();

			/* =========================================================================== */
			/*                             骨格検出&動作認識                               */
			/* =========================================================================== */
			pcl::PointXYZRGB leftFoot, rightFoot, footPoint, basePoint;

			kinect.setSkeleton();
			for (auto person : kinect.skeleton)
			{
				for (auto joint : person)
				{
					if (joint.TrackingState == TrackingState_NotTracked) continue;
					// 左足
					if (joint.JointType == JointType_FootLeft)
					{
						leftFoot.x = joint.Position.X;
						leftFoot.y = joint.Position.Y;
						leftFoot.z = joint.Position.Z;
					}
					// 右足
					if (joint.JointType == JointType_FootRight)
					{
						rightFoot.x = joint.Position.X;
						rightFoot.y = joint.Position.Y;
						rightFoot.z = joint.Position.Z;

					}
					// 左足と右足の中間
					if (leftFoot.z != 0 && rightFoot.z != 0)
					{
						footPoint.x = (leftFoot.x + rightFoot.x) / 2;
						footPoint.y = (leftFoot.y + rightFoot.y) / 2;
						footPoint.z = (leftFoot.z + rightFoot.z) / 2;
					}
					// 腰
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
			/*                                   点郡処理                                  */
			/* =========================================================================== */
			boost::mutex::scoped_try_lock lock(mutex);
			if ((cloud->size() != 0) && lock.owns_lock())
			{
				// 法線推定
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
				
				// 床の特定(法線ベクトル)
				if (floorDetect == false)
				{
					// 複数平面検出
					planeClouds = planesSegmentation(proCloud, 50, 0.035, 0.4);
					//drawClusterPointCloud(viewer, planeClouds, "planes", v1);
					// 高さが最小の平面とその重心抽出
					detectYminPlane(planeClouds, floorCloud, &centroid);
					// 接触判定
					footTouchCloud = RadiusSearch(floorCloud, 128.0f, footPoint, 0.075f);
					//viewer.addText("Touch Cloud Points : " + std::to_string(footTouchCloud->points.size()), 10, 30, "touch points", v1);
					if (footTouchCloud->points.size() != 0)
					{
						// 床の法線ベクトル取得
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

					// 床を基準に補正
					Eigen::Vector3f floor(-1 * floorCoeff->values[0], -1 * floorCoeff->values[1], -1 * floorCoeff->values[2]);
					Eigen::Quaternionf rotate = Eigen::Quaternionf::FromTwoVectors(floor, Eigen::Vector3f::UnitY());
					Eigen::DiagonalMatrix<float, 3> scaling = Eigen::Scaling(1.0f, 1.0f, 1.0f);
					//Eigen::Translation<float, 3> translation(0, kinect.floor.w, 0);
					//Eigen::Affine3f floorMatrix = translation * scaling * rotate;
					//pcl::transformPointCloud(*cloud, *cloud, floorMatrix);
					//viewer.addPointCloud(cloud, "transformed cloud", v1);

					// kinectの高さの計算
					Eigen::Translation<float, 3> trans1(0, 0, 0);
					Eigen::Affine3f AffineMatrix1 = trans1 * scaling * rotate;
					pcl::PointXYZRGB centroidPoint;
					centroidPoint = centroidToPoint(centroid);
					pcl::PointCloud<PointTypeRGB>::Ptr transCloud(new pcl::PointCloud<PointTypeRGB>);
					transCloud->push_back(centroidPoint);
					//viewer.addText("Centroid Y " + std::to_string(-1 * transCloud->points[0].y), 10, 10, "centroid y", v1);
					pcl::transformPointCloud(*transCloud,  *transCloud, AffineMatrix1);
					//viewer.addText("After Centroid Y " + std::to_string(-1 * transCloud->points[0].y), 10, 30, "after centroid y", v1);
					
					// 補正
					Eigen::Translation<float, 3> trans2(0, -1 * transCloud->points[0].y, 0);
					Eigen::Affine3f AffineMatrix2 = trans2 * scaling * rotate;
					pcl::transformPointCloud(*chairCloud, *chairCloud, AffineMatrix2);
					pcl::transformPointCloudWithNormals(*RGBNormalCloud, *RGBNormalCloud, AffineMatrix2);
					pcl::copyPointCloud(*RGBNormalCloud, *tmpCloud);
					pcl::copyPointCloud(*RGBNormalCloud, *normalCloud);
					
					// クラスタリング
					clusterClouds = newEuclideanCluster(tmpCloud, normalCloud, 0.02f, 100, 500);
					//drawClusterPointCloud(viewer, clusterClouds, "cluster", v1);
					viewer.addText("Cluster Numbers : " + std::to_string(clusterClouds.size()), 10, 30, "cluster num", v1);

					// 平面検出
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

							// 床と平行か判定
							double innerPro = CalcuInnerPro(floorCoeff, coefficients);
							if (innerPro >= 0.8 || innerPro <= -0.8)
							{
								// 平行平面抽出
								extractIndices(inCloud, inliers, false);
								pcl::PointCloud<PointTypeRGB>::Ptr planePointCloud(new pcl::PointCloud<PointTypeRGB>);
								*planePointCloud = *inCloud;
								parallelPlaneClouds.push_back(planePointCloud);
								// 重心算出
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
						// 座った時の腰の座標を記録
						pcl::PointCloud<PointTypeRGB>::Ptr baseTransCloud(new pcl::PointCloud<PointTypeRGB>);
						baseTransCloud->push_back(sitBasePoint);
						pcl::transformPointCloud(*baseTransCloud, *baseTransCloud, AffineMatrix2);
						sitHistPoint = baseTransCloud->points[0];
					}
					viewer.addSphere(sitHistPoint, 0.02, 0, 255, 255, "basePoint", v1);
					viewer.addText("Sit Point X : " + std::to_string(sitHistPoint.x) + " Y : " + std::to_string(sitHistPoint.y) + " Z : " + std::to_string(sitHistPoint.z), 10, 70, "sitBasePoint", v1);

					// 腰の座標と接触している平面を抽出
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

						// イス領域の抽出
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

