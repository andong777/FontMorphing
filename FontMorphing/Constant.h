#pragma once

#include <string>
#include <limits>
using namespace std;

static const int infinity = numeric_limits<int>::max();

static const string charImageSuffix = "_R.bmp";
static const string strokeImageSuffix = "F.bmp";
static const string charBoxSuffix = "_R.txt";
static const string strokeBoxSuffix = "_S.txt";

static string sourceCharDir = "E:\\TestData\\PKU\\HT";
static string targetCharDir = "E:\\TestData\\PKU\\LS";
static string charListPath = "E:\\TestData\\PKU\\HT\\list.txt";
static string outputCharDir = "E:\\output";