#pragma once

#include <string>
#include <limits>

//#define CPD_FROM_FILE
//#define VERBOSE
//#define PARALLEL_MODE

using namespace std;

static const int infinity = numeric_limits<int>::max();

static const string charImageSuffix = "_R.bmp";
static const string strokeImageSuffix = "F.bmp";
static const string secondStrokeImageSuffix = ".bmp";
static const string charBoxSuffix = "_R.txt";
static const string strokeBoxSuffix = "_S.txt";

extern string sourceCharDir;
extern string targetCharDir;
extern string charListPath;
extern string outputCharDir;

static int kCPDIterateTimes = 30;
static int kNumSampleStd = 5;
static int kNumSample = 15;
static int kKeyPointsInterval = 100;

// 300px: 100 15; 1000px: 100 5;