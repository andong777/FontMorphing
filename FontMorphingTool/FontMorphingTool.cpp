#pragma once

#include "Constant.h"
#include "DataStructure.h"
#include "CharacterImage.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include "OriCPD.h"
#include "Fade_2D.h"
#include "ARAPMorphingService.h"
#include "PointsPlotDisplayService.h"
#include "MeshPlotDisplayService.h"
#include "Utility.h"
#include <queue>
#include <functional>
#define CPD_FROM_FILE

using namespace FM;
using namespace cv;
using namespace std;
using namespace SGCPD;
using namespace GEOM_FADE2D;
using namespace Eigen;

int numSample = 8;
int numStroke;
CharacterImage sourceChar;
CharacterImage targetChar;
TriMesh triMesh;
vector<int> strokeEndAtVertex;
PointSet sourceCharVert;
PointSet targetCharVert;

void readCharactersData(){
	// read the labels of strokes
	ifstream f;
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
	sourceChar.setNumOfStroke(numStroke);
	targetChar.setNumOfStroke(numStroke);

	// show the character 1
	Mat sourceCharImage = imread(sourceCharPath + "_R.bmp");
	imshow("source", sourceCharImage); waitKey();
	// read the coordinates of the bounding box for a character 视图，骨架边界？
	int sourceCharBox[4];
	f.open(sourceCharPath + charBoxSuffix);
	if (f){
		for (int i = 0; i < 4; i++)	f >> sourceCharBox[i];
		f.close();
	}
	sourceChar.setCharBox(sourceCharBox);
	// read the coordinates of the bounding boxs for all strokes in a given character
	f.open(sourceCharPath + strokeBoxSuffix);
	if (f){
		for (int i = 0; i < numStroke; i++){
			int strokeBox[4];
			for (int j = 0; j < 4; j++)	f >> strokeBox[j];
			sourceChar.setStrokeBox(i, strokeBox);
		}
		f.close();
	}

	// show the character 2
	Mat targetCharImage = imread(targetCharPath + "_R.bmp");
	imshow("target", targetCharImage); waitKey();
	// read the coordinates of the bounding box for a character 视图，骨架边界？
	int targetCharBox[4];
	f.open(targetCharPath + charBoxSuffix);
	if (f){
		for (int i = 0; i<4; i++)	f >> targetCharBox[i];
		f.close();
	}
	targetChar.setCharBox(targetCharBox);
	// read the coordinates of the bounding boxs for all strokes in a given character
	f.open(targetCharPath + strokeBoxSuffix);
	if (f){
		for (int i = 0; i < numStroke; i++){
			int strokeBox[4];
			for (int j = 0; j < 4; j++)	f >> strokeBox[j];
			targetChar.setStrokeBox(i, strokeBox);
		}
		f.close();
	}
}

void getTemplateFromSourceCharacter(){
	// generate the source character by assembling its strokes
	int imgH = sourceChar.getCharHeight() + 10;
	int imgW = sourceChar.getCharWidth() + 10;
	Mat rgb(imgH, imgW, CV_8UC3, Scalar(255, 255, 255));
	for(int no = 0;no < numStroke; no++){
		int offsetX = sourceChar.getStrokeOffsetX(no) + 5;
		int offsetY = sourceChar.getStrokeOffsetY(no) + 5;
		ostringstream os;
		os << sourceCharPath << "_" << (no+1) << ".bmp";
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

	strokeEndAtVertex.resize(numStroke, 0);
	for(int no = 0; no < numStroke; no++){
		cout << "processing stroke "<<no<<endl;
		int offsetX = sourceChar.getStrokeOffsetX(no);
		int offsetY = sourceChar.getStrokeOffsetY(no);
		ostringstream os;
		os << sourceCharPath << "_" << (no+1) << ".bmp";
		Mat strokeImg = imread(os.str(), CV_LOAD_IMAGE_GRAYSCALE);
		Mat largerStrokeImg(strokeImg.size().height+10, strokeImg.size().width+10, CV_8UC1, Scalar(0));
		for(int i=0;i<strokeImg.rows;i++){
			for(int j=0;j<strokeImg.cols;j++){
				largerStrokeImg.at<uchar>(i+5, j+5) = 255 - strokeImg.at<uchar>(i, j);
			}
		}
		Mat edge = getEdge(largerStrokeImg);	// 边缘检测得到的矩阵，边缘为白色。

		PointSet corners;
		vector< vector<bool> > cornerFlag = getCorners(edge, corners);
		for (auto it = corners.begin(); it != corners.end(); it++){
			(*it).x += offsetX;
			(*it).y += offsetY;
		}
		DisplayService *plt = new PointsPlotDisplayService(corners, 5);
		plt->setDisplay("source", Size(0, 0), rgb, Scalar(255, 0, 0));
		plt->doDisplay();
		delete plt;

		// contour line connection method
		PointSet polygon;	//记录多边形上的点
		// 寻找多边形的起始点
		Point startPoint = findStartPoint(edge);
		int currentX = startPoint.x, currentY = startPoint.y;
		cout << "initial point = (" << currentX << ", " << currentY << ")" << endl;
		getPolygon(edge, startPoint, polygon);
		/*for (int i = 0; i < polygon.size(); i++){
			Point p = polygon[i];
			p.x += offsetX;
			p.y += offsetY;
			circle(rgb, p, 1, Scalar(0, 0, 0), CV_FILLED);
		}
		imshow("source", rgb); waitKey();*/

		double sampleInterval = 100.;	// 每两个关键点间的大致距离
		int numKeyPoints = floor(polygon.size() / sampleInterval);	// 根据上面距离，求出关键点个数，即采样段的个数
		double extraInterval = polygon.size() - sampleInterval * numKeyPoints;	// 划分后多余的长度，被均分到每个采样段中
		sampleInterval += extraInterval / numKeyPoints;
		double sampleStep = sampleInterval / numSample;	// 每个采样段中，两个普通点之间的距离
		PointSet samplePoints;	// 采样后得到的点
		for (int i = 0; i < numKeyPoints; i++){
			double basePosition = sampleInterval * i;
			for (int j = 0; j < numSample; j++){
				int pointIndex = floor(basePosition + sampleStep * j);
				samplePoints.push_back(polygon[pointIndex]);
				// replace regular sample points by the corner points
				for (int p = 1; p <= sampleStep - 1; p++){
					int index = floor(fmod(basePosition + sampleStep * (j - .5) + p + polygon.size(), polygon.size()));
					Point pnt = polygon[index];
					if (cornerFlag[pnt.y][pnt.x]){
						cout << "replace corner (" << pnt.x + offsetX << ", " << pnt.y + offsetY << ")" << endl;
						cornerFlag[pnt.y][pnt.x] = false;
						samplePoints.pop_back();
						samplePoints.push_back(pnt);
						break;
					}
				}
			}
		}
		// Plot the polygon
		for (int i = 0; i < samplePoints.size(); i++){	// large stroke image -> character image
			Point *p = &samplePoints[i];
			p->x += offsetX;
			p->y += offsetY;
		}
		DisplayService *plot = new PointsPlotDisplayService(samplePoints, 2, true);
		plot->setDisplay("source", Size(0, 0), rgb);
		plot->doDisplay();
		delete plot;
		for (int i = 0; i < samplePoints.size();i+=numSample){
			circle(rgb, samplePoints[i], 5, Scalar(0, 0, 255));
		}
		imshow("source", rgb); waitKey();
		
		// Triangulation
		vector<Point2> inputPoints;
		for (auto it = samplePoints.begin(); it != samplePoints.end(); it++){
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
		for (auto it = outputTriangles.begin(); it != outputTriangles.end(); it++){
			Point p[3];
			int v[3];
			for (int i = 0; i < 3; i++){
				p[i] = Fade2DPointToCVPoint(*(**it).getCorner(i));
				auto pos = std::find(samplePoints.begin(), samplePoints.end(), p[i]);
				if (pos != samplePoints.end()){
					v[i] = pos - samplePoints.begin();
					v[i] += sourceCharVert.size();	// 加上前面所有笔画的点数，作为三角形顶点的偏移量。注意要在更新点之前更新！
				}
				else{
					v[i] = -1;
					cout << "find triangle index failed." << endl;
				}
			}
			Triangle tri(v[0], v[1], v[2]);
			triMesh.push_back(tri);
		}
		sourceCharVert.insert(sourceCharVert.end(), samplePoints.begin(), samplePoints.end());
		strokeEndAtVertex[no] = sourceCharVert.size() - 1;
 	} // end of for stroke

	for (int i = 0; i < numStroke; i++){
		cout << "stroke " << i << " ends at vertex " << strokeEndAtVertex[i] << endl;
	}
	cout << endl << "There are " << sourceCharVert.size() << " points in source character." << endl;
	// get the connection triangles.
	DisplayService *plot = new MeshPlotDisplayService(triMesh, sourceCharVert);
	plot->setDisplay("source", Size(0, 0), rgb, Scalar(255, 0, 0));
	plot->doDisplay();
	TriMesh connectTri = getConnectTri(sourceCharVert, strokeEndAtVertex);
	cout << "There are " << connectTri.size() << " connection triangles in source character." << endl;
	triMesh.insert(triMesh.end(), connectTri.begin(), connectTri.end());
	cout << "There are " << triMesh.size() << " triangles in source character." << endl;
	plot = new MeshPlotDisplayService(connectTri, sourceCharVert);
	plot->setDisplay("source", Size(0, 0), rgb, Scalar(0, 255, 0));
	plot->doDisplay();
	delete plot;
}

void registerPointSetToTargetCharacter(){
	// generate the target character by assembling its strokes
	int imgH = targetChar.getCharHeight() + 10;
	int imgW = targetChar.getCharWidth() + 10;
	Mat rgb(imgH, imgW, CV_8UC3, Scalar(255, 255, 255));
	for (int no = 0; no < numStroke; no++){
		int offsetX = targetChar.getStrokeOffsetX(no) + 5;
		int offsetY = targetChar.getStrokeOffsetY(no) + 5;
		ostringstream os;
		os << targetCharPath << "_" << (no+1) << ".bmp";
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

	for (int no = 0; no < numStroke; no++){
		cout << "processing stroke " << no << endl;
		int offsetX = targetChar.getStrokeOffsetX(no);
		int offsetY = targetChar.getStrokeOffsetY(no);
		ostringstream os;
		os << targetCharPath << "_" << (no+1) << ".bmp";
		Mat strokeImg = imread(os.str(), CV_LOAD_IMAGE_GRAYSCALE);
		Mat largerStrokeImg(strokeImg.size().height + 10, strokeImg.size().width + 10, CV_8UC1, Scalar(0));
		for (int i = 0; i<strokeImg.rows; i++){
			for (int j = 0; j<strokeImg.cols; j++){
				largerStrokeImg.at<uchar>(i + 5, j + 5) = 255 - strokeImg.at<uchar>(i, j);
			}
		}
		Mat edge = getEdge(largerStrokeImg);

		// find corner points on the contour
		PointSet corners;
		vector< vector<bool> > cornerFlag = getCorners(edge, corners);
		for (auto it = corners.begin(); it != corners.end(); it++){
			(*it).x += offsetX;
			(*it).y += offsetY;
		}
		DisplayService *plt = new PointsPlotDisplayService(corners, 5);
		plt->setDisplay("target", Size(0, 0), rgb, Scalar(255, 0, 0));
		plt->doDisplay();
		delete plt;

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
		for (int i = (no == 0 ? 0 : strokeEndAtVertex[no - 1] + 1); i <= strokeEndAtVertex[no]; i++){
			CPointDouble cpdp = CVPointToSGCPDPoint(sourceCharVert[i]);
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
		PointSet keyPoints;
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
		delete[] vertexBelongToTemplate;
		cout << keyPoints.size() << " key points." << endl;
		for (auto it = keyPoints.begin(); it != keyPoints.end(); it++){
			cout << "(" << (*it).x << ", " << (*it).y << ") ";
			circle(rgb, *it, 5, Scalar(0, 0, 255));
		}
		cout << endl;

		// contour line connection method
		PointSet polygon;	//记录多边形上的点
		for (int i = 0; i<keyPoints.size(); i++){	// character image -> large stroke image
			keyPoints[i].x -= offsetX;
			keyPoints[i].y -= offsetY;
		}
		vector< vector<int> > orderAtThisPoint = getPolygon(edge, keyPoints[0], polygon);	// 记录点在多边形上的顺序
		for (int i = 0; i < polygon.size(); i++){
			Point p = polygon[i];
			p.x += offsetX;
			p.y += offsetY;
			circle(rgb, p, 1, Scalar(0, 0, 0), CV_FILLED);
		}
		imshow("target", rgb); waitKey();

		PointSet samplePoints;	// 采样后得到的点
		for (int i = 0; i<keyPoints.size(); i++){
			int i2 = (i + 1) % keyPoints.size();	// 下一个关键点
			samplePoints.push_back(keyPoints[i]);
			int kp1Order = orderAtThisPoint[keyPoints[i].y][keyPoints[i].x];
			int kp2Order = orderAtThisPoint[keyPoints[i2].y][keyPoints[i2].x];
			if (kp1Order < 0 || kp2Order < 0){
				cout << "some key points are off the polygon." << endl;
				break;
			}
			int dist = abs(kp1Order - kp2Order);
			if ((polygon.size() - dist) >= dist){	// 小圈
				double numIntervalPoints = double(dist) / numSample;	// 中间插点的间隔
				if (kp1Order > kp2Order)	numIntervalPoints *= -1;
				for (int j = 1; j<numSample; j++){
					int index = floor(kp1Order + numIntervalPoints * j);
					samplePoints.push_back(polygon[index]);
					// replace regular sample points by the corner points
					int startPos, endPos;	// to let startPos < endPos
					if (kp1Order < kp2Order){	// 点1 -> 点2
						startPos = floor(kp1Order + numIntervalPoints * (j - .5) + 1);	// notice the number '1', which means it's an exclusive interval. 3.28
						endPos = floor(kp1Order + numIntervalPoints * (j + .5) - 1);	// shorten the search interval. 4.13
					}
					else{	// 点2 -> 点1
						startPos = floor(kp1Order + numIntervalPoints * (j + .5) + 1);
						endPos = floor(kp1Order + numIntervalPoints * (j - .5) - 1);
					}
					for (int p = startPos; p <= endPos; p++){
						Point pnt = polygon[p];
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
					samplePoints.push_back(polygon[(index + polygon.size()) % polygon.size()]);
					// replace regular sample points by the corner points
					for (int p = 1; p <= floor(abs(numIntervalPoints)) - 1; p++){
						index = floor(fmod(kp1Order + numIntervalPoints*(j - .5) + p*step + polygon.size(), polygon.size()));
						Point pnt = polygon[(index + polygon.size()) % polygon.size()];
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
		for (int i = 0; i < samplePoints.size(); i++){	// large stroke image -> character image
			Point *p = &samplePoints[i];
			p->x += offsetX;
			p->y += offsetY;
		}
		DisplayService *plot = new PointsPlotDisplayService(samplePoints);
		plot->setDisplay("target", Size(0, 0), rgb);
		plot->doDisplay();
		delete plot;
		targetCharVert.insert(targetCharVert.end(), samplePoints.begin(), samplePoints.end());
	}
	cout << endl << "There are " << targetCharVert.size() << " points in target character." << endl;
	DisplayService *plot = new MeshPlotDisplayService(triMesh, targetCharVert);
	plot->setDisplay("target", Size(0, 0), rgb, Scalar(255, 0, 0));
	plot->doDisplay();
	delete plot;
}

int main( int, char** argv )
{
	readCharactersData();
	getTemplateFromSourceCharacter();
	registerPointSetToTargetCharacter();
	do{
		ARAPMorphingService *morphing = new ARAPMorphingService;
		int imgH = sourceChar.getCharHeight() + 40;
		int imgW = sourceChar.getCharWidth() + 40;
		morphing->setDisplay("morphing", Size(imgW, imgH));
		morphing->doMorphing(sourceCharVert, targetCharVert, triMesh, 10);
		morphing->doDisplay();
		delete morphing;
	} while (1);
	system("pause");
	return 0;
}