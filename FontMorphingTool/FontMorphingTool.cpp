#include "stdafx.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include "Constant.h"
#include "OriCPD.h"
#include "Fade_2D.h"
#include <Eigen/Dense>

#define CPD_FROM_FILE
#define SHOW_DETAILS

using namespace cv;
using namespace std;
using namespace SGCPD;
using namespace GEOM_FADE2D;
using namespace Eigen;

static const int deltaAngle[8][2] = {{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}};	// 顺时针

Point SGCPDPointToCVPoint(CPointDouble& cpdp);
CPointDouble CVPointToSGCPDPoint(Point& cvp);

Point Fade2DPointToCVPoint(Point2& f2dp);
Point2 CVPointToFade2DPoint(Point& cvp);

Mat fillHoles(Mat imInput);

typedef struct Triangle {
public:
	int va;
	int vb;
	int vc;
	Triangle(){}
	Triangle(int a, int b, int c){
		va = a; vb = b; vc = c;
	}
} Triangle;

ifstream f;

// template data
int numStroke;
vector<Triangle> connectTri;
vector<Point> templateCharVert;	// subscript from 1
int* strokeStartAtVertex;	// must be global!	// todo: use vector instead
int numSample = 8;

// source char
Mat sourceCharImage;
int sourceCharBox[4];
int (*sourceStrokeBox)[4];
vector<Point> sourceCharVert;	// subscript from 1
vector<Triangle> sourceCharTri;

// target char
Mat targetCharImage;
int targetCharBox[4];
int (*targetStrokeBox)[4];
vector<Point> targetCharVert;	// subscript from 1
vector<Triangle> targetCharTri;

void readTemplateCharData(){
	// read the labels of strokes
	f.open(templateCharPathPrefix + "\\" + charName + strokeLabelSuffix);
	if(f){
		string line;
		while(getline(f, line)){
			int num = atoi(line.c_str());
			if(num)
				numStroke++;
		}
		cout << "numStroke: " << numStroke << endl;
		f.close();
	}
	// read the vertex Number of connection triangles for all strokes in a given character %连接三角形所有顶点数（关键点）
	f.open(templateCharPathPrefix + "\\" + charName + connTriSuffix);
	if(f){
		string line;
		int cnt = 0;
		while(getline(f, line)){
			istringstream is(line);
			int a, b, c;
			is >> a >> b >> c;
			Triangle t(a, b, c);
			connectTri.push_back(t);
			cnt++;
		}
		cout << "connectTri num: " << cnt << endl;
		f.close();
	}

	// read stroke meshes and build the whole mesh for the template
	int numVex = 0, numTri;
	strokeStartAtVertex = new int[numStroke + 2]; // starts from 1.
	strokeStartAtVertex[0] = 1;	// vertex number starts from 1. 
	templateCharVert.push_back(Point(0, 0));	// never access subscript 0.
	for(int no=1;no<=numStroke;no++){
 		strokeStartAtVertex[no] = strokeStartAtVertex[no-1] + numVex;
		ostringstream os;
		os << templateCharPath << "_" << no << ".off";
		f.open(os.str().c_str());
		if(f){
			f >> numVex >> numTri >> numSample;	// 每个off文件都存储一个numSample值，看样子可以为不同的笔画指定不同的值，但实际上后来的计算都是用了最后一次循环读出来的numSample。
			for(int i=1;i<=numVex;i++){
				int x, y;
				f >> x >> y;
				Point v(x, y);
				templateCharVert.push_back(v);
			}
			int offset = strokeStartAtVertex[no] - 1;
			f.close();
		}
	}
	strokeStartAtVertex[numStroke+1] = strokeStartAtVertex[numStroke] + numVex;	// 这样我们不需要记录笔画到哪个点结束，只要找下个点开始的位置-1即可。
}

void readSourceCharData(){
	// show the character 1
	sourceCharImage = imread(sourceCharPath + "_R.bmp");
	imshow("source", sourceCharImage); waitKey();
	// read the coordinates of the bounding box for a character 视图，骨架边界？
	f.open(sourceCharPath + charBoxSuffix);
	if(f){
		for(int i=0;i<4;i++){
			f >> sourceCharBox[i];
			cout << sourceCharBox[i] << endl;
		}
		f.close();
	}
	// read the coordinates of the bounding boxs for all strokes in a given character
	f.open(sourceCharPath + strokeBoxSuffix);
	sourceStrokeBox = new int[numStroke+1][4];
	if(f){
		for(int i=1;i<=numStroke;i++){
			for(int j=0;j<4;j++){
				f >> sourceStrokeBox[i][j];
			}
		}
		f.close();
	}
}

void readTargetCharData(){
	// show the character 2
	targetCharImage = imread(targetCharPath + "_R.bmp");
	imshow("target", targetCharImage); waitKey();
	// read the coordinates of the bounding box for a character 视图，骨架边界？
	f.open(targetCharPath + charBoxSuffix);
	if(f){
		for(int i=0;i<4;i++){
			f >> targetCharBox[i];
			cout << targetCharBox[i] << endl;
		}
		f.close();
	}
	// read the coordinates of the bounding boxs for all strokes in a given character
	f.open(targetCharPath + strokeBoxSuffix);
	targetStrokeBox = new int[numStroke+1][4];
	if(f){
		for(int i=1;i<=numStroke;i++){
			for(int j=0;j<4;j++){
				f >> targetStrokeBox[i][j];
			}
		}
		f.close();
	}
}

void registerSourceChar(){
	// generate the source character by assembling its strokes
	int imgH = sourceCharBox[3] - sourceCharBox[1] + 10;
	int imgW = sourceCharBox[2] - sourceCharBox[0] + 10;
	Mat rgb(imgH, imgW, CV_8UC3, Scalar(255, 255, 255));
	for(int no=1;no<=numStroke;no++){
		int offsetX = sourceStrokeBox[no][0] - sourceCharBox[0] + 5;
		int offsetY = sourceStrokeBox[no][1] - sourceCharBox[1] + 5;
		ostringstream os;
		os << sourceCharPath << "_" << no << ".bmp";
		Mat strokeImg = imread(os.str(), CV_LOAD_IMAGE_GRAYSCALE);
		for(int h=0;h<strokeImg.rows;h++){
			for(int w=0;w<strokeImg.cols;w++){
				uchar v = strokeImg.at<uchar>(h, w);
				if(v < 128){	// if black
					rgb.at<Vec3b>(h+offsetY, w+offsetX) = Vec3b(0, 0, 0);
				}
			}
		}
	}

	sourceCharVert.push_back(Point(0, 0));	// never access subscript 0.
	for(int no=1;no<=numStroke;no++){
		cout << "processing stroke "<<no<<endl;
		int offsetX = sourceStrokeBox[no][0] - sourceCharBox[0];
		int offsetY = sourceStrokeBox[no][1] - sourceCharBox[1];
		ostringstream os;
		os << sourceCharPath << "_" << no << ".bmp";
		Mat strokeImg = imread(os.str(), CV_LOAD_IMAGE_GRAYSCALE);
		Mat largerStrokeImg(strokeImg.size().height+10, strokeImg.size().width+10, CV_8UC1, Scalar(0));
		for(int i=0;i<strokeImg.rows;i++){
			for(int j=0;j<strokeImg.cols;j++){
				largerStrokeImg.at<uchar>(i+5, j+5) = 255 - strokeImg.at<uchar>(i, j);
			}
		}
 		Mat filled = fillHoles(largerStrokeImg);
		Mat edge;	// 边缘检测得到的矩阵，边缘为白色。
		Canny(filled, edge, 40, 120);	// Canny方法输出灰度矩阵

		// CPD registration
		vector<CPointDouble> X;
		vector<CPointDouble> Y;
		vector<int> S;
		for(int i=0;i<edge.rows;i++){
			for(int j=0;j<edge.cols;j++){
				if(edge.at<uchar>(i, j) > 128){
					CPointDouble cpdp(j+offsetX, i+offsetY);
					X.push_back(cpdp); 
				}
			}
		}

#ifdef CPD_FROM_FILE
		ostringstream oss;
		oss << "Source_Y_" << no << ".txt";
		ifstream inf(oss.str(), ios::in);
		int Y_size;
		inf >> Y_size;
		for(int i=0;i<Y_size;i++){
			double x, y;
			inf >> x >> y;
			Y.push_back(CPointDouble(x, y));
		}
		inf.close();
#else
		for (int i = strokeStartAtVertex[no]; i<strokeStartAtVertex[no + 1] - 1; i++){
			CPointDouble cpdp = CVPointToSGCPDPoint(templateCharVert[i]);
			Y.push_back(cpdp);
			S.push_back(no);	// or any other constant
		}
		CPD *cpd = new ::SGCPD::OriCPD();
		cpd->SetData(X, Y, S);
		cpd->NormalizeInput();
		cpd->CPDRegister(2, 25, 2, 8, 1e-10, 0.7, 0);
		cpd->DenormalizeOutput();
		vector< vector<CPointDouble> > result = cpd->GetDecompositionResult();
		Y = cpd->GetTemplatePoints();
		delete cpd;
		ostringstream oss;
		oss << "Source_Y_" << no << ".txt";
		ofstream outf(oss.str(), ios::out);
		outf << Y.size() << endl;
		for(int i=0;i<Y.size();i++){
			outf << Y[i].x << " " << Y[i].y << endl;
		}
		outf.close();
#endif

		for(int i=0;i<Y.size();i++){
			circle(rgb, SGCPDPointToCVPoint(Y[i]), 5, Scalar(0, 255, 0));
		}

		// find key points on the contour
		vector<Point> keyPoints;
		int *vertexBelongToTemplate = new int[X.size()];
		for(int i=0;i<X.size();i++)	vertexBelongToTemplate[i] = -1;
		for(int i=0;i<Y.size();i+=numSample){
			int minDist = infinity, dist, index;
			for(int j=0;j<X.size();j++){
				if(vertexBelongToTemplate[j] < 0){	// 尚未确定该点的对应关系
					dist = (X[j].x-Y[i].x)*(X[j].x-Y[i].x)+(X[j].y-Y[i].y)*(X[j].y-Y[i].y);
					if(dist < minDist){
						minDist = dist;
						index = j;
					}
				}
			}
			vertexBelongToTemplate[index] = i;
			Point p(X[index].x, X[index].y);
			keyPoints.push_back(p);
		}
		cout << keyPoints.size() << " key points." << endl;
		for (vector<Point>::iterator it = keyPoints.begin(); it != keyPoints.end(); it++){
			cout << "(" << (*it).x << ", " << (*it).y << ") ";
			circle(rgb, *it, 5, Scalar(0, 0, 255));
		}
		cout << endl;

		// find corner points on the contour
		bool **cornerFlag = new bool*[edge.size().height];
		for(int i=0;i<edge.size().height;i++){
			cornerFlag[i] = new bool[edge.size().width];
		}
		for(int i=0;i<edge.size().height;i++){
			for(int j=0;j<edge.size().width;j++){
				cornerFlag[i][j] = false;
			}
		}
		vector<Point> corners;
		goodFeaturesToTrack(edge, corners, 20, .05, 20, Mat(), 5, true, .1);
		cout << corners.size() << " corner points." << endl;
		for(int i=0;i<corners.size();i++){
			cornerFlag[corners[i].y][corners[i].x] = true;
			corners[i].x += offsetX;
			corners[i].y += offsetY;
			cout << "(" << corners[i].x << ", " << corners[i].y << ") ";
			circle(rgb, corners[i], 5, Scalar(255, 0, 0));
		}
		imshow("source", rgb); waitKey();

		// contour line connection method
		vector<Point> polygon;	//记录多边形上的点
		int **orderAtThisPoint = new int*[edge.size().height];	// 记录点在多边形上的顺序，从1开始
		for(int i=0;i<edge.size().height;i++){
			orderAtThisPoint[i] = new int[edge.size().width];
		}
		for(int i=0;i<strokeImg.size().height;i++){
			for(int j=0;j<strokeImg.size().width;j++){
				orderAtThisPoint[i][j] = 0;
			}
		}
		for(int i=0;i<keyPoints.size();i++){	// char image -> large stroke image
			keyPoints[i].x -= offsetX;
			keyPoints[i].y -= offsetY;
		}
		int currentX = keyPoints[0].x;
		int currentY = keyPoints[0].y;
		while(edge.at<uchar>(currentY, currentX) > 128){
			polygon.push_back(Point(currentX, currentY));
			orderAtThisPoint[currentY][currentX] = polygon.size();
			edge.at<uchar>(currentY, currentX) = 0;	// 标记此点已经用过，避免找回到此点
			int firstStepCnt = 0;
			int secondStepCnt[8] = {0};	// 标记选择一个点后，第二次能找到几个点
			int firstStepDirection[8] = {0};
			for(int i=0;i<8;i++){
				int tempX = currentX + deltaAngle[i][0];
				int tempY = currentY + deltaAngle[i][1];
				if(edge.at<uchar>(tempY, tempX) > 128){
					firstStepCnt++;
					firstStepDirection[firstStepCnt-1] = i;
					for(int j=0;j<8;j++){
						int tempXX = tempX + deltaAngle[j][0];
						int tempYY = tempY + deltaAngle[j][1];
						if(edge.at<uchar>(tempYY, tempXX) > 128){
							secondStepCnt[firstStepCnt-1]++;
						}
					}
				}
			}
			if(firstStepCnt == 1){	// 只找到了一个后续点
				currentX += deltaAngle[firstStepDirection[0]][0];
				currentY += deltaAngle[firstStepDirection[0]][1];
			}else if(firstStepCnt > 1){	// 多个可选的后续点
				int minCnt = infinity, direct = -1;
				for(int i=0;i<firstStepCnt;i++){
					if(secondStepCnt[i] >= 1 && secondStepCnt[i] < minCnt){	// 找后续点最少的后续点？
						minCnt = secondStepCnt[i];
						direct = firstStepDirection[i];
					}
				}
				currentX += deltaAngle[direct][0];
				currentY += deltaAngle[direct][1];
			}else{
				cout << "find no next points" << endl;
			}
		}
		vector<Point> samplePoints;	// 采样后得到的点
		for(int i=0;i<keyPoints.size();i++){
			int i2 = (i + 1) % keyPoints.size();	// 下一个关键点
			samplePoints.push_back(keyPoints[i]);
			int kp1Order = orderAtThisPoint[keyPoints[i].y][keyPoints[i].x];
			int kp2Order = orderAtThisPoint[keyPoints[i2].y][keyPoints[i2].x];
			if(kp1Order <= 0 || kp2Order <= 0){
				cout << "some key points are off the polygon." << endl;
				break;
			}
			int dist = abs(kp1Order - kp2Order);
			if((polygon.size() - dist) >= dist){	// 小圈
				double numIntervalPoints = double(dist) / numSample;	// 中间插点的间隔
				if (kp1Order > kp2Order)	numIntervalPoints *= -1;
				for(int j=1;j<numSample;j++){
					int index = floor(kp1Order + numIntervalPoints * j);
					samplePoints.push_back(polygon[index-1]);	// 因为order是从1开始，所以取下标要减1.
					// replace regular sample points by the corner points
					int startPos, endPos;	// to let startPos < endPos
					if(kp1Order < kp2Order){	// 点1 -> 点2
						startPos = floor(kp1Order + numIntervalPoints * (j - 1) + 1);	// notice the number '1', 
						endPos = floor(kp1Order + numIntervalPoints * (j + 1) - 1);	// which means it's an exclusive interval. 3.28
					}else{	// 点2 -> 点1
						startPos = floor(kp1Order + numIntervalPoints * (j + 1) + 1);
						endPos = floor(kp1Order + numIntervalPoints * (j - 1) - 1);
					}
					for(int p=startPos;p<=endPos;p++){
						Point pnt = polygon[p - 1];	// 同上
						if(cornerFlag[pnt.y][pnt.x]){
							cout << "小圈 replace corner ("<<pnt.x+offsetX<<", "<<pnt.y+offsetY<<")"<<endl;
							cornerFlag[pnt.y][pnt.x] = false;
							samplePoints.pop_back();
							samplePoints.push_back(pnt);
							break;
						}
					}
				}
			}else{	// 大圈
				double numIntervalPoints = double(polygon.size() - dist) / numSample;
				if(kp1Order < kp2Order)	numIntervalPoints *= -1;	// 注意这里是小于而不是大于
				int step = numIntervalPoints / abs(numIntervalPoints);	// +1 or -1
				for(int j=1;j<numSample;j++){
					int index = floor(fmod(kp1Order + numIntervalPoints*j + polygon.size(), polygon.size())); // notice the detail
					samplePoints.push_back(polygon[(index-1+polygon.size())%polygon.size()]);
					// replace regular sample points by the corner points
					int numSampleSub = 2 * floor(abs(numIntervalPoints)) - 1;
					for(int p=1;p<=numSampleSub;p++){
						index = floor(fmod(kp1Order + numIntervalPoints*(j - 1) + p*step + polygon.size(), polygon.size()));
						Point pnt = polygon[(index - 1 + polygon.size()) % polygon.size()];
						if(cornerFlag[pnt.y][pnt.x]){
							cout << "大圈 replace corner ("<<pnt.x+offsetX<<", "<<pnt.y+offsetY<<")"<<endl;
							cornerFlag[pnt.y][pnt.x] = false;
							samplePoints.pop_back();
							samplePoints.push_back(pnt);
							break;
						}
					}
				}
			}
		}

		// Plot the polygon
		cout << samplePoints.size() << " sample points." << endl;
		for(int i=0;i<samplePoints.size();i++){
			Point *p = &samplePoints[i];
			p->x += offsetX;
			p->y += offsetY;
			circle(rgb, *p, 2, Scalar(0, 0, 0), CV_FILLED);
		}
		imshow("source", rgb); waitKey();
		
		// Triangulation
		vector<Point2> inputPoints;
		for (vector<Point>::iterator it = samplePoints.begin(); it != samplePoints.end(); it++){
			inputPoints.push_back(CVPointToFade2DPoint(*it));
		}
		Fade_2D dt;
		dt.insert(inputPoints);
		vector<Segment2> segments;
		for (int i = 0; i < inputPoints.size(); i++){
			Segment2 seg(inputPoints[i], inputPoints[(i + 1)%inputPoints.size()]);
			segments.push_back(seg);
		}
		ConstraintGraph2* constraint = dt.createConstraint(segments, CIS_KEEP_DELAUNAY);
		Zone2* zone = dt.createZone(constraint, ZL_INSIDE);
		dt.applyConstraintsAndZones();
		vector<Triangle2*> outputTriangles;
		zone->getTriangles(outputTriangles);
		
		// 更新变量，释放内存等
		for (vector<Triangle2*>::iterator it = outputTriangles.begin(); it != outputTriangles.end(); it++){
			Point p[3];
			int v[3];
			for (int i = 0; i < 3; i++){
				p[i] = Fade2DPointToCVPoint(*(**it).getCorner(i));
				vector<Point>::iterator pos = std::find(samplePoints.begin(), samplePoints.end(), p[i]);
				if (pos != samplePoints.end()){
					v[i] = pos - samplePoints.begin() + 1;	// vertices starts from 1.
					v[i] += sourceCharVert.size() - 1;	// 加上前面所有笔画的点数，作为三角形顶点的偏移量。注意要在更新点之前更新！
				}
				else{
					v[i] = -1;
					cout << "find triangle index failed." << endl;
				}
			}
			Triangle tri(v[0], v[1], v[2]);
			sourceCharTri.push_back(tri);
			for (int i = 0; i < 3; i++){
				line(rgb, p[i], p[(i + 1) % 3], Scalar(255, 0, 0));
			}
		}
		sourceCharVert.insert(sourceCharVert.end(), samplePoints.begin(), samplePoints.end());
		imshow("source", rgb); waitKey();

		delete[] vertexBelongToTemplate;
		for(int i=0;i<edge.size().height;i++){
			delete[] cornerFlag[i];
			delete[] orderAtThisPoint[i];
		}
		delete[] cornerFlag;
		delete[] orderAtThisPoint;
 	}
	for (vector<Triangle>::iterator it = connectTri.begin(); it != connectTri.end(); it++){
		int a = (*it).va;	// because vertex starts from 1.
		int b = (*it).vb;
		int c = (*it).vc;
		line(rgb, sourceCharVert[a], sourceCharVert[b], Scalar(255, 0, 0));
		line(rgb, sourceCharVert[b], sourceCharVert[c], Scalar(255, 0, 0));
		line(rgb, sourceCharVert[c], sourceCharVert[a], Scalar(255, 0, 0));
	}
	imshow("source", rgb); waitKey();
	sourceCharTri.insert(sourceCharTri.end(), connectTri.begin(), connectTri.end());
	cout << endl << "There are " << sourceCharVert.size() << " points in source character." << endl;
	cout << endl << "There are " << sourceCharTri.size() << " triangles in source character." << endl;
}

void registerTargetChar(){
	// generate the target character by assembling its strokes
	int imgH = targetCharBox[3] - targetCharBox[1] + 10;
	int imgW = targetCharBox[2] - targetCharBox[0] + 10;
	Mat rgb(imgH, imgW, CV_8UC3, Scalar(255, 255, 255));
	for (int no = 1; no <= numStroke; no++){
		int offsetX = targetStrokeBox[no][0] - targetCharBox[0] + 5;
		int offsetY = targetStrokeBox[no][1] - targetCharBox[1] + 5;
		ostringstream os;
		os << targetCharPath << "_" << no << ".bmp";
		Mat strokeImg = imread(os.str(), CV_LOAD_IMAGE_GRAYSCALE);
		for (int h = 0; h<strokeImg.rows; h++){
			for (int w = 0; w<strokeImg.cols; w++){
				uchar v = strokeImg.at<uchar>(h, w);
				if (v < 128){	// if black
					rgb.at<Vec3b>(h + offsetY, w + offsetX) = Vec3b(0, 0, 0);
				}
			}
		}
	}
	targetCharVert.push_back(Point(0, 0));	// never access subscript 0.
	for (int no = 1; no <= numStroke; no++){
		cout << "processing stroke " << no << endl;
		int offsetX = targetStrokeBox[no][0] - targetCharBox[0];
		int offsetY = targetStrokeBox[no][1] - targetCharBox[1];
		ostringstream os;
		os << targetCharPath << "_" << no << ".bmp";
		Mat strokeImg = imread(os.str(), CV_LOAD_IMAGE_GRAYSCALE);
		Mat largerStrokeImg(strokeImg.size().height + 10, strokeImg.size().width + 10, CV_8UC1, Scalar(0));
		for (int i = 0; i<strokeImg.rows; i++){
			for (int j = 0; j<strokeImg.cols; j++){
				largerStrokeImg.at<uchar>(i + 5, j + 5) = 255 - strokeImg.at<uchar>(i, j);
			}
		}
		Mat filled = fillHoles(largerStrokeImg);
		Mat edge;	// 边缘检测得到的矩阵，边缘为白色。
		Canny(filled, edge, 40, 120);	// Canny方法输出灰度矩阵
		//imshow("", edge); waitKey();
		// CPD registration
		vector<CPointDouble> X;
		vector<CPointDouble> Y;
		vector<int> S;
		for (int i = 0; i<edge.rows; i++){
			for (int j = 0; j<edge.cols; j++){
				if (edge.at<uchar>(i, j) > 128){
					CPointDouble cpdp(j + offsetX, i + offsetY);
					X.push_back(cpdp);
				}
			}
		}

#ifdef CPD_FROM_FILE
		ostringstream oss;
		oss << "Target_Y_" << no << ".txt";
		ifstream inf(oss.str(), ios::in);
		int Y_size;
		inf >> Y_size;
		for (int i = 0; i<Y_size; i++){
			double x, y;
			inf >> x >> y;
			Y.push_back(CPointDouble(x, y));
		}
		inf.close();
#else
		for (int i = strokeStartAtVertex[no]; i<strokeStartAtVertex[no + 1] - 1; i++){
			CPointDouble cpdp = CVPointToSGCPDPoint(templateCharVert[i]);
			Y.push_back(cpdp);
			S.push_back(no);	// or any other constant
		}
		CPD *cpd = new ::SGCPD::OriCPD();
		cpd->SetData(X, Y, S);
		cpd->NormalizeInput();
		cpd->CPDRegister(2, 25, 2, 8, 1e-10, 0.7, 0);
		cpd->DenormalizeOutput();
		vector< vector<CPointDouble> > result = cpd->GetDecompositionResult();
		Y = cpd->GetTemplatePoints();
		delete cpd;
		ostringstream oss;
		oss << "Target_Y_" << no << ".txt";
		ofstream outf(oss.str(), ios::out);
		outf << Y.size() << endl;
		for(int i=0;i<Y.size();i++){
			outf << Y[i].x << " " << Y[i].y << endl;
		}
		outf.close();
#endif
		for (int i = 0; i<Y.size(); i++){
			circle(rgb, SGCPDPointToCVPoint(Y[i]), 5, Scalar(0, 255, 0));
		}

		// find key points on the contour
		vector<Point> keyPoints;
		int *vertexBelongToTemplate = new int[X.size()];
		for (int i = 0; i<X.size(); i++)	vertexBelongToTemplate[i] = -1;
		for (int i = 0; i<Y.size(); i += numSample){
			int minDist = infinity, dist, index;
			for (int j = 0; j<X.size(); j++){
				if (vertexBelongToTemplate[j] < 0){	// 尚未确定该点的对应关系
					dist = (X[j].x - Y[i].x)*(X[j].x - Y[i].x) + (X[j].y - Y[i].y)*(X[j].y - Y[i].y);
					if (dist < minDist){
						minDist = dist;
						index = j;
					}
				}
			}
			vertexBelongToTemplate[index] = i;
			Point p(X[index].x, X[index].y);
			keyPoints.push_back(p);
		}
		cout << keyPoints.size() << " key points." << endl;
		for (vector<Point>::iterator it = keyPoints.begin(); it != keyPoints.end(); it++){
			cout << "(" << (*it).x << ", " << (*it).y << ") ";
			circle(rgb, *it, 5, Scalar(0, 0, 255));
		}
		cout << endl;

		// find corner points on the contour
		bool **cornerFlag = new bool*[edge.size().height];
		for (int i = 0; i<edge.size().height; i++){
			cornerFlag[i] = new bool[edge.size().width];
		}
		for (int i = 0; i<edge.size().height; i++){
			for (int j = 0; j<edge.size().width; j++){
				cornerFlag[i][j] = false;
			}
		}
		vector<Point> corners;
		goodFeaturesToTrack(edge, corners, 20, .05, 20, Mat(), 5, true, .1);
		cout << corners.size() << " corner points." << endl;
		for (int i = 0; i<corners.size(); i++){
			cornerFlag[corners[i].y][corners[i].x] = true;
			corners[i].x += offsetX;
			corners[i].y += offsetY;
			circle(rgb, corners[i], 5, Scalar(255, 0, 0));
		}
		imshow("target", rgb); waitKey();

		// contour line connection method
		vector<Point> polygon;	//记录多边形上的点
		int **orderAtThisPoint = new int*[edge.size().height];	// 记录点在多边形上的顺序
		for (int i = 0; i<edge.size().height; i++){
			orderAtThisPoint[i] = new int[edge.size().width];
		}
		for (int i = 0; i<strokeImg.size().height; i++){
			for (int j = 0; j<strokeImg.size().width; j++){
				orderAtThisPoint[i][j] = 0;
			}
		}
		for (int i = 0; i<keyPoints.size(); i++){	// char image -> large stroke image
			keyPoints[i].x -= offsetX;
			keyPoints[i].y -= offsetY;
		}
		int currentX = keyPoints[0].x;
		int currentY = keyPoints[0].y;
		while (edge.at<uchar>(currentY, currentX) > 128){
			polygon.push_back(Point(currentX, currentY));
			orderAtThisPoint[currentY][currentX] = polygon.size();
			edge.at<uchar>(currentY, currentX) = 0;	// 标记此点已经用过，避免找回到此点
			int firstStepCnt = 0;
			int secondStepCnt[8] = { 0 };	// 标记选择一个点后，第二次能找到几个点
			int firstStepDirection[8] = { 0 };
			for (int i = 0; i<8; i++){
				int tempX = currentX + deltaAngle[i][0];
				int tempY = currentY + deltaAngle[i][1];
				if (edge.at<uchar>(tempY, tempX) > 128){
					firstStepCnt++;
					firstStepDirection[firstStepCnt - 1] = i;
					for (int j = 0; j<8; j++){
						int tempXX = tempX + deltaAngle[j][0];
						int tempYY = tempY + deltaAngle[j][1];
						if (edge.at<uchar>(tempYY, tempXX) > 128){
							secondStepCnt[firstStepCnt - 1]++;
						}
					}
				}
			}
			if (firstStepCnt == 1){	// 只找到了一个后续点
				currentX += deltaAngle[firstStepDirection[0]][0];
				currentY += deltaAngle[firstStepDirection[0]][1];
			}
			else if (firstStepCnt > 1){	// 多个可选的后续点
				int minCnt = infinity, direct = -1;
				for (int i = 0; i<firstStepCnt; i++){
					if (secondStepCnt[i] >= 1 && secondStepCnt[i] < minCnt){	// 找后续点最少的后续点？
						minCnt = secondStepCnt[i];
						direct = firstStepDirection[i];
					}
				}
				currentX += deltaAngle[direct][0];
				currentY += deltaAngle[direct][1];
			}
			else{
				cout << "find no next points" << endl;
			}
		}
		vector<Point> samplePoints;	// 采样后得到的点
		for (int i = 0; i<keyPoints.size(); i++){
			int i2 = (i + 1) % keyPoints.size();	// 下一个关键点
			samplePoints.push_back(keyPoints[i]);
			int kp1Order = orderAtThisPoint[keyPoints[i].y][keyPoints[i].x];
			int kp2Order = orderAtThisPoint[keyPoints[i2].y][keyPoints[i2].x];
			if (kp1Order <= 0 || kp2Order <= 0){
				cout << "some key points are off the polygon." << endl;
				break;
			}
			int dist = abs(kp1Order - kp2Order);
			if ((polygon.size() - dist) >= dist){	// 小圈
				double numIntervalPoints = double(dist) / numSample;	// 中间插点的间隔
				if (kp1Order > kp2Order)	numIntervalPoints *= -1;
				for (int j = 1; j<numSample; j++){
					int index = floor(kp1Order + numIntervalPoints * j);
					samplePoints.push_back(polygon[index - 1]);	// 因为order是从1开始，所以取下标要减1.
					// replace regular sample points by the corner points
					int startPos, endPos;	// to let startPos < endPos
					if (kp1Order < kp2Order){	// 点1 -> 点2
						startPos = floor(kp1Order + numIntervalPoints * (j - 1) + 1);	// notice the number '1', 
						endPos = floor(kp1Order + numIntervalPoints * (j + 1) - 1);	// which means it's an exclusive interval. 3.28
					}
					else{	// 点2 -> 点1
						startPos = floor(kp1Order + numIntervalPoints * (j + 1) + 1);
						endPos = floor(kp1Order + numIntervalPoints * (j - 1) - 1);
					}
					for (int p = startPos; p <= endPos; p++){
						Point pnt = polygon[p - 1];	// 同上
						if (cornerFlag[pnt.y][pnt.x]){
							cout << "小圈 replace corner (" << pnt.x + offsetX << ", " << pnt.y + offsetY << ")" << endl;
							cornerFlag[pnt.y][pnt.x] = false;
							samplePoints.pop_back();
							samplePoints.push_back(pnt);
							break;
						}
					}
				}
			}
			else{	// 大圈
				double numIntervalPoints = double(polygon.size() - dist) / numSample;
				if (kp1Order < kp2Order)	numIntervalPoints *= -1;	// 注意这里是小于而不是大于
				int step = numIntervalPoints / abs(numIntervalPoints);	// +1 or -1
				for (int j = 1; j<numSample; j++){
					int index = floor(fmod(kp1Order + numIntervalPoints*j + polygon.size(), polygon.size())); // notice the detail
					samplePoints.push_back(polygon[(index - 1 + polygon.size()) % polygon.size()]);
					// replace regular sample points by the corner points
					int numSampleSub = 2 * floor(abs(numIntervalPoints)) - 1;
					for (int p = 1; p <= numSampleSub; p++){
						index = floor(fmod(kp1Order + numIntervalPoints*(j - 1) + p*step + polygon.size(), polygon.size()));
						Point pnt = polygon[(index - 1 + polygon.size()) % polygon.size()];
						if (cornerFlag[pnt.y][pnt.x]){
							cout << "大圈 replace corner (" << pnt.x + offsetX << ", " << pnt.y + offsetY << ")" << endl;
							cornerFlag[pnt.y][pnt.x] = false;
							samplePoints.pop_back();
							samplePoints.push_back(pnt);
							break;
						}
					}
				}
			}
		}

		// Plot the polygon
		cout << samplePoints.size() << " sample points." << endl;
		for (int i = 0; i<samplePoints.size(); i++){
			Point *p = &samplePoints[i];
			p->x += offsetX;
			p->y += offsetY;
			circle(rgb, *p, 2, Scalar(0, 0, 0), CV_FILLED);
		}
		imshow("target", rgb); waitKey();

		// Triangulation
		vector<Point2> inputPoints;
		for (vector<Point>::iterator it = samplePoints.begin(); it != samplePoints.end(); it++){
			inputPoints.push_back(CVPointToFade2DPoint(*it));
		}
		Fade_2D dt;
		dt.insert(inputPoints);
		vector<Segment2> segments;
		for (int i = 0; i < inputPoints.size(); i++){
			Segment2 seg(inputPoints[i], inputPoints[(i + 1) % inputPoints.size()]);
			segments.push_back(seg);
		}
		ConstraintGraph2* constraint = dt.createConstraint(segments, CIS_KEEP_DELAUNAY);
		Zone2* zone = dt.createZone(constraint, ZL_INSIDE);
		dt.applyConstraintsAndZones();
		vector<Triangle2*> outputTriangles;
		zone->getTriangles(outputTriangles);
		
		// 更新变量，释放内存等
		for (vector<Triangle2*>::iterator it = outputTriangles.begin(); it != outputTriangles.end(); it++){
			Point p[3];
			int v[3];
			for (int i = 0; i < 3; i++){
				p[i] = Fade2DPointToCVPoint(*(**it).getCorner(i));
				vector<Point>::iterator pos = std::find(samplePoints.begin(), samplePoints.end(), p[i]);
				if (pos != samplePoints.end()){
					v[i] = pos - samplePoints.begin() + 1;
					v[i] += targetCharVert.size() - 1;
				}
				else{
					v[i] = -1;
					cout << "find triangle index failed." << endl;
				}
			}
			Triangle tri(v[0], v[1], v[2]);
			targetCharTri.push_back(tri);
			for (int i = 0; i < 3; i++){
				line(rgb, p[i], p[(i + 1) % 3], Scalar(255, 0, 0));
			}
		}
		targetCharVert.insert(targetCharVert.end(), samplePoints.begin(), samplePoints.end());
		imshow("target", rgb); waitKey();

		delete[] vertexBelongToTemplate;
		for (int i = 0; i<edge.size().height; i++){
			delete[] cornerFlag[i];
			delete[] orderAtThisPoint[i];
		}
		delete[] cornerFlag;
		delete[] orderAtThisPoint;
	}
	for (vector<Triangle>::iterator it = connectTri.begin(); it != connectTri.end(); it++){
		int a = (*it).va;	// because vertex starts from 1.
		int b = (*it).vb;
		int c = (*it).vc;
		line(rgb, targetCharVert[a], targetCharVert[b], Scalar(255, 0, 0));
		line(rgb, targetCharVert[b], targetCharVert[c], Scalar(255, 0, 0));
		line(rgb, targetCharVert[c], targetCharVert[a], Scalar(255, 0, 0));
	}
	imshow("target", rgb); waitKey();
	targetCharTri.insert(targetCharTri.end(), connectTri.begin(), connectTri.end());
	cout << endl << "There are " << targetCharVert.size() << " points in target character." << endl;
	cout << endl << "There are " << targetCharTri.size() << " triangles in target character." << endl;
}

void doMorphing(int numStep){
	int posFixed = 10;
	vector<Triangle> &tri = sourceCharTri;
	int numTri = tri.size(), numVert = templateCharVert.size() - 1;
	Point *sv = new Point[numVert + 1];
	Point *tv = new Point[numVert + 1];
	for (int i = 0; i <= numVert; i++){
		sv[i] = sourceCharVert[i] - sourceCharVert[posFixed];
		tv[i] = targetCharVert[i] - targetCharVert[posFixed];
	}

	float *angle = new float[numTri];
	for (int i = 0; i < numTri; i++) angle[i] = .0;
	vector<MatrixXf> D;
	MatrixXf H = MatrixXf::Zero(2 * numVert, 2 * numVert);
	vector<MatrixXf> G_temp;
	for (int i = 0; i < numTri; i++){
		Triangle t = tri[i];
		MatrixXf H_temp = MatrixXf::Zero(4, 2 * numVert);
		Point triSource[] = { sv[t.va], sv[t.vb], sv[t.vc] };
		Point triTarget[] = { tv[t.va], tv[t.vb], tv[t.vc] };
		MatrixXf Al(6, 6);
		Al << triSource[0].x, triSource[0].y, 0, 0, 1, 0,
			0, 0, triSource[0].x, triSource[0].y, 0, 1,
			triSource[1].x, triSource[1].y, 0, 0, 1, 0,
			0, 0, triSource[1].x, triSource[1].y, 0, 1,
			triSource[2].x, triSource[2].y, 0, 0, 1, 0,
			0, 0, triSource[2].x, triSource[2].y, 0, 1;
		VectorXf b(6);
		b << triTarget[0].x, triTarget[0].y, triTarget[1].x, triTarget[1].y, triTarget[2].x, triTarget[2].y;
		VectorXf C = Al.colPivHouseholderQr().solve(b);
		MatrixXf Al_temp = Al.inverse().topRows(4);
		H_temp.col(2 * t.va - 1 - 1) = Al_temp.col(0);
		H_temp.col(2 * t.va - 1) = Al_temp.col(1);
		H_temp.col(2 * t.vb - 1 - 1) = Al_temp.col(2);
		H_temp.col(2 * t.vb - 1) = Al_temp.col(3);
		H_temp.col(2 * t.vc - 1 - 1) = Al_temp.col(4);
		H_temp.col(2 * t.vc - 1) = Al_temp.col(5);
		H += H_temp.transpose() * H_temp;
		G_temp.push_back(H_temp.transpose());
		Matrix2f A_ori;
		A_ori << C(0), C(1), C(2), C(3);
		JacobiSVD<MatrixXf> svd(A_ori, ComputeFullU | ComputeFullV);
		Matrix2f S = Matrix2f::Zero();
		VectorXf singularValues = svd.singularValues();
		for (int j = 0; j < singularValues.rows(); j++){
			S(j, j) = singularValues(j);
		}
		MatrixXf U = svd.matrixU();
		MatrixXf V = svd.matrixV();  
		MatrixXf Rr = U * V.transpose();
		D.push_back(V * S * V.transpose());
		angle[i] = asinf(Rr(1, 0));
		if (Rr(0, 0) < 0 && Rr(1, 0) > 0){
			angle[i] = acosf(Rr(0, 0));
		}
		else if (Rr(0, 0) < 0 && Rr(1, 0) < 0){
			angle[i] = -angle[i] - M_PI;
		}
	}

	posFixed--;
	MatrixXf tempMatH(2 * numVert - 2, 2 * numVert - 2);
	int longEdge = 2 * posFixed, shortEdge = 2 * numVert - 2 * posFixed - 2;
	tempMatH.topLeftCorner(longEdge, longEdge) = H.topLeftCorner(longEdge, longEdge);
	tempMatH.bottomLeftCorner(shortEdge, longEdge) = H.bottomLeftCorner(shortEdge, longEdge);
	tempMatH.topRightCorner(longEdge, shortEdge) = H.topRightCorner(longEdge, shortEdge);
	tempMatH.bottomRightCorner(shortEdge, shortEdge) = H.bottomRightCorner(shortEdge, shortEdge);
	MatrixXf H_new = tempMatH.inverse();

	for (int i = 0; i <= numStep; i++){
		float ratio = 1.f * i / numStep;
		VectorXf G = VectorXf::Zero(2 * numVert);
		for (int j = 0; j < numTri; j++){
			Matrix2f Rr;
			float tmpAngle = angle[j] * ratio;
			Rr << cosf(tmpAngle), -sinf(tmpAngle), sinf(tmpAngle), cosf(tmpAngle);
			Matrix2f A = Rr * ((1 - ratio) * Matrix2f::Identity() + ratio * D[j]);
			Vector4f A_temp(A(0, 0), A(0, 1), A(1, 0), A(1, 1));
			G += G_temp[j] * A_temp;
		}
		VectorXf tempVecG(2 * numVert - 2);
		tempVecG.head(longEdge) = G.head(longEdge);
		tempVecG.tail(shortEdge) = G.tail(shortEdge);
		VectorXf xy = H_new * tempVecG;
		Point *tempVert = new Point[numVert + 1];
		tempVert[posFixed + 1] = sv[posFixed + 1];
		for (int j = 1; j <= posFixed; j++){
			int x = xy(j * 2 - 1 - 1);
			int y = xy(j * 2 - 1);
			tempVert[j] = Point(x, y);
		}
		for (int j = posFixed + 2; j <= numVert; j++){
			int x = xy(j * 2 - 3 - 1);
			int y = xy(j * 2 - 2 - 1);
			tempVert[j] = Point(x, y);
		}
		for (int j = 1; j <= numVert; j++){
			tempVert[j].x += sourceCharVert[posFixed].x + 20;
			tempVert[j].y += sourceCharVert[posFixed].y + 20;
		}

		/*ostringstream osss;
		osss << "TempVert_step_" << i << ".txt";*/

		/*ofstream outf(osss.str(), ios::out);
		for (int j = 1; j<=numVert; j++){
			outf << tempVert[j].x << " " << tempVert[j].y << endl;
		}
		outf.close();*/

		/*ifstream inf(osss.str(), ios::in);
		for (int j = 1; j<=numVert; j++){
			double x, y;
			inf >> x >> y;
			tempVert[j].x = x;
			tempVert[j].y = y;
		}
		inf.close();*/

		int imgH = sourceCharBox[3] - sourceCharBox[1] + 10;
		int imgW = sourceCharBox[2] - sourceCharBox[0] + 10;
		Mat image(floor(1.f*imgH) + 30, floor(1.f*imgW) + 30, CV_8UC1, Scalar(255));
		for (int i = 0; i < numTri/* - connectTri.size()*/; i++){
#ifdef SHOW_DETAILS
			int a = tri[i].va;
			int b = tri[i].vb;
			int c = tri[i].vc;
			line(image, tempVert[a], tempVert[b], Scalar(0));
			line(image, tempVert[b], tempVert[c], Scalar(0));
			line(image, tempVert[c], tempVert[a], Scalar(0));
#else
			Point gon[1][3];
			gon[0][0] = tempVert[tri[i].va];
			gon[0][1] = tempVert[tri[i].vb];
			gon[0][2] = tempVert[tri[i].vc];
			const Point* ppt[1] = { gon[0] }; int npt[] = { 3 };
			fillPoly(image, ppt, npt, 1, Scalar(0, 0, 0));
#endif
		}
		delete tempVert;
		imshow("morphing", image); waitKey();
		ostringstream oss;
		oss << charName << "_" << i << ".jpg";
		imwrite(oss.str(), image);
	}
	delete angle, tv, sv;
}

int main( int, char** argv )
{
	readTemplateCharData();
	readSourceCharData();
	readTargetCharData();
	registerSourceChar();
	registerTargetChar();
	do{
		doMorphing(20);
	} while (1);
	system("pause");
	return 0;
}

Point SGCPDPointToCVPoint(CPointDouble& cpdp){
	Point cvp(cpdp.x, cpdp.y);
	return cvp;
}

CPointDouble CVPointToSGCPDPoint(Point& cvp){
	CPointDouble cpdp(cvp.x, cvp.y);
	return cpdp;
}

Point Fade2DPointToCVPoint(Point2& f2dp){
	Point cvp(f2dp.x(), f2dp.y());
	return cvp;
}

Point2 CVPointToFade2DPoint(Point& cvp){
	Point2 f2dp(cvp.x, cvp.y);
	return f2dp;
}

Mat fillHoles(Mat imInput){
    Mat imShow = Mat::zeros(imInput.size(),CV_8UC3);    // for show result
	vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(imInput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (int idx=0;idx < contours.size();idx++){
        drawContours(imShow,contours,idx,Scalar::all(255),CV_FILLED,8);
    }
    Mat imFilledHoles;
    cvtColor(imShow,imFilledHoles,CV_BGR2GRAY);
    imFilledHoles = imFilledHoles > 0;
    imShow.release();
    return imFilledHoles;
}