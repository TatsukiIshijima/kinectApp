#include "stdafx.h"
// C��min,max�}�N���𖳌��ɂ���
#define NOMINMAX
// �G���[�𖳎�����
#pragma warning(disable:4996)

#include <pcl\io\io.h>

// �_�S�̌^���`
typedef pcl::PointXYZ PointType;                                                                      // XYZ���W
typedef pcl::PointXYZRGB PointTypeRGB;                                                                // XYZ���W + �F
typedef pcl::Normal NormalType;                                                                       // �@��
typedef pcl::Label LabelType;
typedef pcl::PointXYZRGBNormal PointTypeRGBNormal;
