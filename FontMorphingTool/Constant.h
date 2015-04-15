#pragma once

#include <string>
#include <limits>
using namespace std;

static int infinity = numeric_limits<int>::max();

static const string charName = "FM1";
static const string templateCharPathPrefix = "E:\\TestData\\PKU";
static const string sourceCharPathPrefix = "E:\\TestData\\PKU\\HT";
static const string targetCharPathPrefix = "E:\\TestData\\PKU\\LS";

static const string charImageSuffix = "_R.bmp";
static const string strokeLabelSuffix = "_CatParts.txt";
static const string charBoxSuffix = "_R.txt";
static const string strokeBoxSuffix = "_S.txt";
static const string connTriSuffix = "_ConnectTri.txt";

static string templateCharPath = templateCharPathPrefix + "\\" + charName;
static string sourceCharPath = sourceCharPathPrefix + "\\" + charName;
static string targetCharPath = targetCharPathPrefix + "\\" + charName;