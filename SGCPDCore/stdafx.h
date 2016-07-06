// stdafx.h : ��׼ϵͳ�����ļ��İ����ļ���
// ���Ǿ���ʹ�õ��������ĵ�
// �ض�����Ŀ�İ����ļ�
//

#pragma once

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // �� Windows ͷ���ų�����ʹ�õ�����



// TODO: �ڴ˴����ó�����Ҫ������ͷ�ļ�
#include <Eigen/Dense>
using namespace Eigen;

#define POINT_SIZE 2
#define MAX_STROKE_COUNT 100
#define ORIGINAL_CPD 0
#define OPTIMIZED_CPD 1
#define CPD_BATCH_FILE_METHOD OPTIMIZED_CPD

#define DELETE_DATA_THRESHOLD 0
#define DELETE_DATA_WEIGHT 0
#define MAX_POINT_COUNT 10000

// GBRF iteration P optimization related.
#define P_PROMOTION_RATE 1

// threshold for deleting the outliers in local CPDLO.
#define DISTANCE_THRESHOLD 0.03