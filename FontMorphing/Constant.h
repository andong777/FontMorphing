#pragma once

#include <string>
#include <limits>

//#define CPD_FROM_FILE
//#define DEMO_MODE

using namespace std;

static const int infinity = numeric_limits<int>::max();

static const string charImageSuffix = "_R.bmp";
static const string strokeImageSuffix = "F.bmp";
static const string charBoxSuffix = "_R.txt";
static const string strokeBoxSuffix = "_S.txt";

extern string sourceCharDir;
extern string targetCharDir;
extern string charListPath;
extern string outputCharDir;