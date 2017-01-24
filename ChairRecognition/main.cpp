// ChairRecognition_ver2.0.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

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
		viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v1);
		viewer.setBackgroundColor(0, 0, 0, v1);

		int v2(0);
		viewer.createViewPort(0.5, 0.5, 1.0, 1.0, v2);
		viewer.setBackgroundColor(0.2, 0.2, 0.2, v2);

		int v3(0);
		viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v3);
		viewer.setBackgroundColor(0.4, 0.4, 0.4, v3);

		int v4(0);
		viewer.createViewPort(0.5, 0.0, 1.0, 0.5, v4);
		viewer.setBackgroundColor(0.6, 0.6, 0.6, v4);

		viewer.addCoordinateSystem(1.0);

		pcl::PointCloud<PointTypeRGB>::Ptr cloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr copyCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr inCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr outCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<NormalType>::Ptr normalCloud(new pcl::PointCloud<NormalType>);
		pcl::PointCloud<PointTypeRGBNormal>::Ptr RGBNormalCloud(new pcl::PointCloud<PointTypeRGBNormal>);

		pcl::PointXYZRGB sitBasePoint;													// 座った時の腰の座標
		pcl::PointXYZRGB sitHistPoint;													// 座った時の腰の座標の履歴

		std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> clusterClouds;					// クラスタリング点郡
		std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> planeClouds;					// 平面点郡
		std::vector<pcl::PointCloud<PointTypeRGB>::Ptr> parallelPlaneClouds;			// 平行平面点郡

		std::vector<pcl::PointXYZRGB> planeCentroids;									// 平行平面の重心

		double sitScore;																// 座り動作の類似度
		int count = 0;																	// カウント変数

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
			pcl::PointXYZRGB basePoint;

			kinect.setSkeleton();
			for (auto person : kinect.skeleton)
			{
				for (auto joint : person)
				{
					if (joint.TrackingState == TrackingState_NotTracked) continue;

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
					viewer.addText("Sit Score : " + std::to_string(sitScore), 10, 30, "Sit Score", v1);
				}
			}
			
			/* =========================================================================== */
			/*                                   点郡処理                                  */
			/* =========================================================================== */
			boost::mutex::scoped_try_lock lock(mutex);
			if ((cloud->size() != 0) && lock.owns_lock())
			{
				pcl::copyPointCloud(*cloud, *copyCloud);
				passThroughFilter(copyCloud, "z", 0.00, 4.50, true);
				normalCloud = normalEstimationKinect(copyCloud);
				pcl::concatenateFields(*copyCloud, *normalCloud, *RGBNormalCloud);
				voxelGridFilterRGBNormal(RGBNormalCloud, 0.015f);
				
				// 床の法線ベクトルと床からkinectの高さ(W)の取得
				std::string floorParam = "X : " + std::to_string(kinect.floor.x) +
										" , Y : " + std::to_string(kinect.floor.y) +
										" , Z : " + std::to_string(kinect.floor.z) +
										" , W : " + std::to_string(kinect.floor.w);

				if (kinect.floor.x == 0 && kinect.floor.y == 0 && kinect.floor.z == 0 && kinect.floor.w == 0) continue;
				// 床面のパラメータ
				pcl::ModelCoefficients::Ptr floorCoeff(new pcl::ModelCoefficients);
				floorCoeff->values.push_back(kinect.floor.x);
				floorCoeff->values.push_back(kinect.floor.y);
				floorCoeff->values.push_back(kinect.floor.z);
				viewer.addText("Floor Vector " + floorParam, 10, 10, "Floor Vector", v1);

				// 床を基準に点郡を補正
				Eigen::Vector3f floorVec(kinect.floor.x, kinect.floor.y, kinect.floor.z);
				Eigen::Quaternionf rotate = Eigen::Quaternionf::FromTwoVectors(floorVec, Eigen::Vector3f::UnitY());
				Eigen::Translation<float, 3> translation(0, kinect.floor.w, 0);
				Eigen::DiagonalMatrix<float, 3> scaling = Eigen::Scaling(1.0f, 1.0f, 1.0f);
				Eigen::Affine3f floorMatrix = translation * scaling * rotate;
				pcl::transformPointCloudWithNormals(*RGBNormalCloud, *RGBNormalCloud, floorMatrix);
				// 点郡を法線と分離
				pcl::copyPointCloud(*RGBNormalCloud, *copyCloud);
				pcl::copyPointCloud(*RGBNormalCloud, *normalCloud);
				// クラスタリング
				viewer.addPointCloud(copyCloud, "TransformedCloud", v1);
				clusterClouds = newEuclideanCluster(copyCloud, normalCloud, 0.02f, 100, 500);
				drawClusterPointCloud(viewer, clusterClouds, "Cluster", v2);
				viewer.addText("Number of Cluster : " + std::to_string(clusterClouds.size()), 10, 10, "Number of Cluster", v2);
				
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
						// 各クラスターで平面検出
						planeSegmentation(clusterClouds[i], coefficients, inliers, 50, 0.035);
						if (inliers->indices.size() == 0) break;

						// 床面との平行判定
						double innerPro = CalcuInnerPro(floorCoeff, coefficients);
						if (innerPro >= 0.8 || innerPro <= -0.8)
						{
							//std::cout << innerPro << std::endl;
							// 平行平面抽出
							extractIndices(inCloud, inliers, false);
							pcl::PointCloud<PointTypeRGB>::Ptr planePointCloud(new pcl::PointCloud<PointTypeRGB>);
							*planePointCloud = *inCloud;
							parallelPlaneClouds.push_back(planePointCloud);
							// 平行平面の重心算出
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
				drawClusterPointCloud(viewer, parallelPlaneClouds, "ParallelPlaneCloud", v3);
				viewer.addText("Number of ParallelPlane : " + std::to_string(parallelPlaneClouds.size()), 10, 10, "Number of ParallelPlane", v3);
				
				// 座り動作時の腰の座標を記録
				if (sitBasePoint.x != 0 && sitBasePoint.y != 0 && sitBasePoint.z != 0 && sitScore >= 0.50)
				{
					pcl::PointCloud<PointTypeRGB>::Ptr transedBaseCloud(new pcl::PointCloud<PointTypeRGB>);
					transedBaseCloud->push_back(sitBasePoint);
					pcl::transformPointCloud(*transedBaseCloud, *transedBaseCloud, floorMatrix);
					sitHistPoint = transedBaseCloud->points[0];
				}
				viewer.addSphere(sitHistPoint, 0.02, 0, 255, 255, "BasePoint", v1);
				viewer.addText("Sit Point X : " + std::to_string(sitHistPoint.x) + " Y : " + std::to_string(sitHistPoint.y) + " Z : " + std::to_string(sitHistPoint.z), 10, 50, "SitBasePoint", v1);

				// 腰の座標と接触している平面インデックス取得
				int index = getTouchedClsterIndex(parallelPlaneClouds, sitHistPoint, 128.0f, 0.30);
				if (index != NULL)
				{
					// 平面から椅子の領域取得
					pcl::visualization::PointCloudColorHandlerCustom<PointTypeRGB> red(parallelPlaneClouds[index], 255, 0, 0);
					viewer.addPointCloud(parallelPlaneClouds[index], red, "ExtractedPlaneCloud", v1);
					double height = planeCentroids[index].y;
					pcl::PointXYZRGB min_pointAABB;
					pcl::PointXYZRGB max_pointAABB;
					MomentOfInertiaByAABB(parallelPlaneClouds[index], &min_pointAABB, &max_pointAABB);
					double x_min = min_pointAABB.x - 0.10;
					double x_max = max_pointAABB.x + 0.10;
					double y_min = min_pointAABB.y - height + 0.10;
					double y_max = max_pointAABB.y + height + 0.10;
					double z_min = min_pointAABB.z - 0.15;
					double z_max = max_pointAABB.z + 0.15;
					passThroughFilter(copyCloud, "x", x_min, x_max, false);
					passThroughFilter(copyCloud, "y", y_min, y_max, false);
					passThroughFilter(copyCloud, "z", z_min, z_max, false);
					viewer.addCube(x_min, x_max, y_min, y_max, z_min, z_max, 1.0, 1.0, 1.0, "AABB", v1);
					viewer.addPointCloud(copyCloud, "Chair", v4);
					pcl::io::savePCDFileBinary("SaveCloud/chair" + std::to_string(count) + ".pcd", *copyCloud);
					count++;
				}
				parallelPlaneClouds.clear();
				planeCentroids.clear();
			}
			if (GetKeyState(VK_ESCAPE) < 0) break;
		}
		grabber.stop();
	}
	catch (const std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
	}
    return 0;
}

