// CylinderRecognition.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
// 把持前の物体の円柱認識

#include "stdafx.h"
#include <sys/stat.h>
// Cのmin,maxマクロを無効にする
#define NOMINMAX
#define USE_GESTURE

// 安全でないメソッドの呼び出しでの警告を無効にする
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <conio.h>
#include <windows.h>
#include <time.h>
#include <numeric>

#include "kinect2_grabber.h"
#include "PCLNtkinect.h"
#include "PCLConvexhull.h"
#include "PCLFilter.h"
#include "PCLNormal.h"
#include "PCLSearch.h"
#include "PCLSegmentation.h"

#include <pcl\registration\transforms.h>
#include <pcl\segmentation\organized_multi_plane_segmentation.h>
#include <pcl\segmentation\organized_connected_component_segmentation.h>

NtKinect kinect;

double circleDetectforLSM(pcl::PointCloud<PointTypeRGB>::Ptr cloud)
{
	double radius = 0.0;

	pcl::ConvexHull <PointTypeRGB> conv;
	pcl::PointCloud<PointTypeRGB>::Ptr xy_cloud(new pcl::PointCloud<PointTypeRGB>);
	pcl::PointCloud<PointTypeRGB>::Ptr yz_cloud(new pcl::PointCloud<PointTypeRGB>);
	pcl::PointCloud<PointTypeRGB>::Ptr zx_cloud(new pcl::PointCloud<PointTypeRGB>);
	//pcl::PointCloud<PointTypeRGB>::Ptr xy_cloud_vexhull(new pcl::PointCloud<PointTypeRGB>);
	//pcl::PointCloud<PointTypeRGB>::Ptr yz_cloud_vexhull(new pcl::PointCloud<PointTypeRGB>);
	pcl::PointCloud<PointTypeRGB>::Ptr zx_cloud_vexhull(new pcl::PointCloud<PointTypeRGB>);

	// 全体にフィルタ処理
	removalFilter(cloud, 50, 1.0);
	// 平行投影
	perspectiveProjection(cloud, xy_cloud, yz_cloud, zx_cloud);
	
	// 各投影点郡のフィルタ処理
	//removalFilter(xy_cloud, 50, 0.25);
	//removalFilter(yz_cloud, 50, 0.25);
	removalFilter(zx_cloud, 50, 0.25);
	//voxelGridFilter(xy_cloud, 0.0025f);
	//voxelGridFilter(zx_cloud, 0.0025f);
	voxelGridFilter(zx_cloud, 0.0025f);

	// 凸包座標算出
	//conv.setInputCloud(xy_cloud);
	//conv.reconstruct(*xy_cloud_vexhull);

	//conv.setInputCloud(yz_cloud);
	//conv.reconstruct(*yz_cloud_vexhull);

	conv.setInputCloud(zx_cloud);
	conv.reconstruct(*zx_cloud_vexhull);

	// 2次元座標に変換
	std::vector<pcl::PointXY> zxConvexPoints = convert2d(zx_cloud_vexhull);

	// 円フィッティング
	double center_x, center_y;
	fitCircle(zxConvexPoints, &center_x, &center_y, &radius);

	return radius;
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
		
		pcl::PointCloud<PointTypeRGB>::Ptr cloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr extractLeftCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr extractRightCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr removeLeftPlaneCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr removeRightPlaneCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr saveLeftCloud(new pcl::PointCloud<PointTypeRGB>);
		pcl::PointCloud<PointTypeRGB>::Ptr saveRightCloud(new pcl::PointCloud<PointTypeRGB>);

		pcl::ModelCoefficients::Ptr floorCoeff(new pcl::ModelCoefficients);

		double m = 2.50;			// 外分比
		double n = 1.00;			// 外分比
		double length = 0.10;		// 切り取り領域の余分の長さ
		double L_radius = 0.00;		// 半径
		double R_radius = 0.00;

		int count = 1;
		int frameCount = 0;
		bool circleDetect = false;
		
		/* =========================================================================== */
		/*                               kinect準備等                                  */
		/* =========================================================================== */
		kinect.setGestureFile(L"drinkSample.gbd");
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
			viewer.removeAllShapes();
			viewer.removeAllPointClouds();

			/* =========================================================================== */
			/*                                   点郡処理                                  */
			/* =========================================================================== */
			boost::mutex::scoped_try_lock lock(mutex);
			if ((cloud->size() != 0) && lock.owns_lock())
			{
				passThroughFilter(cloud, "z", 0.00, 4.25, false);

				//viewer.addPointCloud(cloud, "cloud", v1);
				//viewer.addText("Number of cloud points : " + std::to_string(cloud->size()), 10, 400, "number of points", v1);

				// 床の法線ベクトルとkinectの高さ(w)
				std::string param = "X : " + std::to_string(kinect.floor.x) +
									" , Y : " + std::to_string(kinect.floor.y) +
									" , Z : " + std::to_string(kinect.floor.z) +
									" , W : " + std::to_string(kinect.floor.w);
				floorCoeff->values.push_back(kinect.floor.x);
				floorCoeff->values.push_back(kinect.floor.y);
				floorCoeff->values.push_back(kinect.floor.z);

				viewer.addText("Floor normal param " + param, 10, 10, "floor param text1", v1);
				//viewer.addText("Floor normal param " + param, 10, 10, "floor param text2", v2);

				// 床を基準に補正
				Eigen::Vector3f floor(kinect.floor.x, kinect.floor.y, kinect.floor.z);
				Eigen::Quaternionf rotate = Eigen::Quaternionf::FromTwoVectors(floor, Eigen::Vector3f::UnitY());
				Eigen::Translation<float, 3> translation(0, kinect.floor.w, 0);
				Eigen::DiagonalMatrix<float, 3> scaling = Eigen::Scaling(1.0f, 1.0f, 1.0f);
				Eigen::Affine3f floorMatrix = translation * scaling * rotate;
				pcl::transformPointCloud(*cloud, *cloud, floorMatrix);

				viewer.addPointCloud(cloud, "cloud", v1);
				viewer.addText("Number of cloud points : " + std::to_string(cloud->size()), 10, 30, "number of points", v1);
				//viewer.addPointCloud(cloud, "filterd Cloud", v2);
				//viewer.addText("Number of cloud points : " + std::to_string(cloud->size()), 10, 400, "after number of points", v2);
				
				if (circleDetect == false)
				{
					/* =========================================================================== */
					/*                                   骨格検出                                  */
					/* =========================================================================== */
					pcl::PointXYZRGB leftHand, leftElbow, leftPoint;
					pcl::PointXYZRGB rightHand, rightElbow, rightPoint;
					//pcl::PointCloud<PointType>::Ptr jointCloud(new pcl::PointCloud<PointType>);
					pcl::PointCloud<PointTypeRGB>::Ptr leftClouds(new pcl::PointCloud<PointTypeRGB>);
					pcl::PointCloud<PointTypeRGB>::Ptr rightClouds(new pcl::PointCloud<PointTypeRGB>);
					pcl::PointCloud<PointTypeRGB>::Ptr leftAreaCloud(new pcl::PointCloud<PointTypeRGB>);
					pcl::PointCloud<PointTypeRGB>::Ptr rightAreaCloud(new pcl::PointCloud<PointTypeRGB>);

					*leftAreaCloud = *cloud;
					*rightAreaCloud = *cloud;

					kinect.setSkeleton();
					for (auto person : kinect.skeleton)
					{
						for (auto joint : person)
						{
							if (joint.TrackingState == TrackingState_NotTracked) continue;
							
							if (joint.JointType == JointType_HandLeft)
							{
								leftHand.x = joint.Position.X;
								leftHand.y = joint.Position.Y;
								leftHand.z = joint.Position.Z;
							}

							if (joint.JointType == JointType_HandRight)
							{
								rightHand.x = joint.Position.X;
								rightHand.y = joint.Position.Y;
								rightHand.z = joint.Position.Z;
							}

							if (joint.JointType == JointType_ElbowLeft)
							{
								leftElbow.x = joint.Position.X;
								leftElbow.y = joint.Position.Y;
								leftElbow.z = joint.Position.Z;
							}

							if (joint.JointType == JointType_ElbowRight)
							{
								rightElbow.x = joint.Position.X;
								rightElbow.y = joint.Position.Y;
								rightElbow.z = joint.Position.Z;
							}
						}
					}

					// 手の点郡を座標変換
					if (leftHand.x != 0 && leftElbow.x != 0 && rightHand.x != 0 && rightElbow.x != 0)
					{
						leftPoint.x = (-n * leftElbow.x + m * leftHand.x) / (m - n);
						leftPoint.y = (-n * leftElbow.y + m * leftHand.y) / (m - n);
						leftPoint.z = (-n * leftElbow.z + m * leftHand.z) / (m - n);
						leftClouds->push_back(leftPoint);
						pcl::transformPointCloud(*leftClouds, *leftClouds, floorMatrix);

						rightPoint.x = (-n * rightElbow.x + m * rightHand.x) / (m - n);
						rightPoint.y = (-n * rightElbow.y + m * rightHand.y) / (m - n);
						rightPoint.z = (-n * rightElbow.z + m * rightHand.z) / (m - n);
						rightClouds->push_back(rightPoint);
						pcl::transformPointCloud(*rightClouds, *rightClouds, floorMatrix);
					}
					/*
					// 左手領域の抽出
					if (leftClouds->size() != 0)
					{
						viewer.addCube(leftClouds->points[0].x - length, leftClouds->points[0].x + length,
									   leftClouds->points[0].y - length, leftClouds->points[0].y + length,
									   leftClouds->points[0].z - length, leftClouds->points[0].z + length,
									   1.0, 1.0, 0.0, "LeftArea1", v1);

						passThroughFilter(leftAreaCloud, "x", leftClouds->points[0].x - length, leftClouds->points[0].x + length, false);
						passThroughFilter(leftAreaCloud, "y", leftClouds->points[0].y - length, leftClouds->points[0].y + length, false);
						passThroughFilter(leftAreaCloud, "z", leftClouds->points[0].z - length, leftClouds->points[0].z + length, false);

						viewer.addText("LeftPosition   X : " + std::to_string(leftClouds->points[0].x) + " Y : " + std::to_string(leftClouds->points[0].y) + " Z : " + std::to_string(leftClouds->points[0].z), 10, 50, "Left Hand Position", v1);
						viewer.addText("Number of LeftAreaCloud : " + std::to_string(leftAreaCloud->size()), 10, 70, "Left Area Cloud", v1);

						// 左手領域内の処理
						if (leftAreaCloud->points.size() >= 500)
						{
							*extractLeftCloud = *leftAreaCloud;
							//removalFilter(extractLeftCloud, 50, 1.0);
							viewer.addCube(leftClouds->points[0].x - length, leftClouds->points[0].x + length,
										   leftClouds->points[0].y - length, leftClouds->points[0].y + length,
										   leftClouds->points[0].z - length, leftClouds->points[0].z + length,
										   1.0, 1.0, 0.0, "LeftArea2", v2);
							viewer.addPointCloud(extractLeftCloud, "Extracted Left Cloud", v2);
							viewer.addText("Number of extracted Left cloud points : " + std::to_string(extractLeftCloud->size()), 10, 10, "Extracted Left Cloud Points", v2);

							// 平行平面除去
							removeLeftPlaneCloud = parallelPlaneRemove(extractLeftCloud, 50, 0.005, 0.3, floorCoeff);
							viewer.addText("Number of removed Left cloud points : " + std::to_string(removeLeftPlaneCloud->size()), 10, 30, "Removed Left Cloud Points", v2);
							// 円検出
							L_radius = circleDetectforLSM(removeLeftPlaneCloud);
							viewer.addText("L_Radius : " + std::to_string(L_radius), 10, 50, "L_Radius", v2);
						}
					}
					*/

					// 右手領域の抽出
					if (rightClouds->size() != 0)
					{
						viewer.addCube(rightClouds->points[0].x - length, rightClouds->points[0].x + length,
							rightClouds->points[0].y - length, rightClouds->points[0].y + length,
							rightClouds->points[0].z - length, rightClouds->points[0].z + length,
							1.0, 1.0, 0.0, "RightArea", v1);

						passThroughFilter(rightAreaCloud, "x", rightClouds->points[0].x - length, rightClouds->points[0].x + length, false);
						passThroughFilter(rightAreaCloud, "y", rightClouds->points[0].y - length, rightClouds->points[0].y + length, false);
						passThroughFilter(rightAreaCloud, "z", rightClouds->points[0].z - length, rightClouds->points[0].z + length, false);

						viewer.addText("RightPosition   X : " + std::to_string(rightClouds->points[0].x) + " Y : " + std::to_string(rightClouds->points[0].y) + " Z : " + std::to_string(rightClouds->points[0].z), 10, 90, "Rigt Hand Position", v1);
						viewer.addText("Number of RightAreaCloud : " + std::to_string(rightAreaCloud->size()), 10, 110, "Right Area Cloud", v1);

						// 右手領域内の処理
						if (rightAreaCloud->points.size() >= 500)
						{
							*extractRightCloud = *rightAreaCloud;
							//removalFilter(extractLeftCloud, 50, 1.0);
							viewer.addCube(rightClouds->points[0].x - length, rightClouds->points[0].x + length,
										   rightClouds->points[0].y - length, rightClouds->points[0].y + length,
										   rightClouds->points[0].z - length, rightClouds->points[0].z + length,
									   	   1.0, 1.0, 0.0, "RightArea2", v2);
							viewer.addPointCloud(extractRightCloud, "Extracted Right Cloud", v2);
							viewer.addText("Number of extracted Right cloud points : " + std::to_string(extractLeftCloud->size()), 10, 70, "Extracted Right Cloud Points", v2);

							// 平行平面除去
							removeRightPlaneCloud = parallelPlaneRemove(extractRightCloud, 50, 0.005, 0.3, floorCoeff);
							viewer.addText("Number of removed Right cloud points : " + std::to_string(removeRightPlaneCloud->size()), 10, 90, "Removed Right Cloud Points", v2);
							// 円検出
							R_radius = circleDetectforLSM(removeRightPlaneCloud);
							viewer.addText("R_Radius : " + std::to_string(R_radius), 10, 110, "R_Radius", v2);
							
							if (R_radius >= 0.03 && R_radius <= 0.05 && removeRightPlaneCloud->points.size() > 400) 
							{
								pcl::copyPointCloud(*removeRightPlaneCloud, *saveRightCloud);
								//pcl::io::savePCDFileBinary("drink" + std::to_string(count) + ".pcd", *saveRightCloud);
								circleDetect = true;
							}
						}
					}
				}

				else
				{
					viewer.addPointCloud(saveRightCloud, "Save Right Cloud", v2);
					viewer.addText("Frame Count" + std::to_string(frameCount), 10, 130, "frameCount", v2);

					/* =========================================================================== */
					/*                                   動作認識                                  */
					/* =========================================================================== */
					kinect.setGesture();
					for (int i = 0; i < kinect.discreteGesture.size(); i++)
					{
						auto g = kinect.discreteGesture[i];

						//viewer.addText(kinect.gesture2string(g.first) + std::to_string(g.second), 10, 130, "drink", v2);
						if (kinect.gesture2string(g.first) == "drinkSample_Right")
						{
							viewer.removeShape("drink", v1);
							viewer.addText("Drink Right: " + std::to_string(g.second), 10, 130, "drink", v1);
							pcl::io::savePCDFileBinary("drink" + std::to_string(count) + ".pcd", *saveRightCloud);
							count++;
							circleDetect = false;
						}
						else if (kinect.gesture2string(g.first) == "drinkSample_Left")
						{
							viewer.removeShape("drink", v1);
							viewer.addText("Drink Left: " + std::to_string(g.second), 10, 150, "drink", v1);
						}
					}
					// フレーム数が120までに飲む動作を行わないときは形状認識に戻る
					frameCount++;
					if (frameCount == 120)
					{
						circleDetect = false;
						frameCount = 0;
					}
				}
			}
			// 床の法線ベクトルのクリア
			floorCoeff->values.clear();

			if (GetKeyState(VK_ESCAPE) < 0)
			{
				//pcl::io::savePCDFileBinary("ModelCloud.pcd", *extractLeftCloud);
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

