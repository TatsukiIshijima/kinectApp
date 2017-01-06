#include "stdafx.h"
#include "PCLPointType.h"
#include <pcl\search\kdtree.h>
#include <pcl\kdtree\kdtree.h>
#include <pcl\kdtree\kdtree_flann.h>
#include <pcl\features\normal_3d_omp.h>
#include <pcl\features\integral_image_normal.h>

// –@ü„’è
pcl::PointCloud<NormalType>::Ptr normalEstimation(pcl::PointCloud<PointTypeRGB>::Ptr cloud);
// Kinect—p–@ü„’è
pcl::PointCloud<NormalType>::Ptr normalEstimationKinect(pcl::PointCloud<PointTypeRGB>::Ptr cloud);