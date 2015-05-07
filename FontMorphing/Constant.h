#pragma once

#include <string>
#include <limits>

#define DEMO_MODE
#define CPD_FROM_FILE
//#define VERBOSE
#define PARALLEL_MODE

#ifdef DEMO_MODE
#undef PARALLEL_MODE
#endif

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

static const int kCPDIterateTimes = 25;
static const int kNumSample = 6;
static const int kKeyPointsInterval = 50;
