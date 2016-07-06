// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // 从 Windows 头中排除极少使用的资料



// TODO: 在此处引用程序需要的其他头文件
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