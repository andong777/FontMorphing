#pragma once

#include <string>
#include <limits>
using namespace std;

static int infinity = numeric_limits<int>::max();

static const string sourceCharDir = "E:\\TestData\\PKU\\HT";
static const string targetCharDir = "E:\\TestData\\PKU\\LS";
static const string outputCharDir = "E:\\output";

static const string charImageSuffix = "_R.bmp";
static const string strokeImageSuffix = ".bmp";
static const string charBoxSuffix = "_R.txt";
static const string strokeBoxSuffix = "_S.txt";