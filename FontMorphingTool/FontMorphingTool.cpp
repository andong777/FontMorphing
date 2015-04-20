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
#include <queue>
#include "OriCPD.h"
#include "Fade_2D.h"
#include "Utility.h"
#include "ARAPMorphing.h"
#include "PointsDisplay.h"
#include "MeshDisplay.h"
#include "CharacterDisplay.h"

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

	// show the character 1
	Mat sourceCharImage = imread(sourceCharPath + "_R.bmp");
	imshow("source", sourceCharImage); waitKey();
	// read the coordinates of the bounding box for a character 视图，骨架边界？
	int x1, y1, x2, y2;
	f.open(sourceCharPath + charBoxSuffix);
	if (f){
		f >> x1 >> y1 >> x2 >> y2;
		f.close();
	}
	sourceChar.setChar(sourceCharImage, Rect(Point(x1, y1), Point(x2, y2)));
	// read the coordinates of the bounding boxs for all strokes in a given character
	f.open(sourceCharPath + strokeBoxSuffix);
	if (f){
		for (int i = 0; i < numStroke; i++){
			ostringstream os;
			os << sourceCharPath << "_" << (i + 1) << ".bmp";
			Mat strokeImg = imread(os.str(), CV_LOAD_IMAGE_GRAYSCALE);
			f >> x1 >> y1 >> x2 >> y2;
			sourceChar.addStroke(strokeImg, Rect(Point(x1, y1), Point(x2, y2)));
		}
		f.close();
	}

	// show the character 2
	Mat targetCharImage = imread(targetCharPath + "_R.bmp");
	imshow("target", targetCharImage); waitKey();
	// read the coordinates of the bounding box for a character 视图，骨架边界？
	f.open(targetCharPath + charBoxSuffix);
	if (f){
		f >> x1 >> y1 >> x2 >> y2;
		f.close();
	}
	targetChar.setChar(targetCharImage, Rect(Point(x1, y1), Point(x2, y2)));
	// read the coordinates of the bounding boxs for all strokes in a given character
	f.open(targetCharPath + strokeBoxSuffix);
	if (f){
		for (int i = 0; i < numStroke; i++){
			ostringstream os;
			os << targetCharPath << "_" << (i + 1) << ".bmp";
			Mat strokeImg = imread(os.str(), CV_LOAD_IMAGE_GRAYSCALE);
			f >> x1 >> y1 >> x2 >> y2;
			targetChar.addStroke(strokeImg, Rect(Point(x1, y1), Point(x2, y2)));
		}
		f.close();
	}
}

void getTemplateFromSourceCharacter(){
	Mat rgb(sourceChar.getCharSize() + Size(10, 10), CV_8UC3, Scalar(255, 255, 255));
	DisplayService *display = new CharacterDisplay(sourceChar);
	display->setDisplay("source", Size(0, 0), rgb, Scalar(0, 0, 0));
	display->doDisplay();
	delete display;

	strokeEndAtVertex.resize(numStroke, 0);
	for(int no = 0; no < numStroke; no++){
		cout << "processing stroke "<<no<<endl;
		Mat& strokeImg = sourceChar.strokeImage.at(no);
		Mat largerStrokeImg(strokeImg.size() + Size(10, 10), CV_8UC1, Scalar(0));
		for(int i=0;i<strokeImg.rows;i++){
			for(int j=0;j<strokeImg.cols;j++){
				largerStrokeImg.at<uchar>(i+5, j+5) = 255 - strokeImg.at<uchar>(i, j);
			}
		}
		Mat edge = getEdge(largerStrokeImg);	// 边缘检测得到的矩阵，边缘为白色。

		PointSet corners;
		Point offset = sourceChar.getStrokeOffset(no);
		vector< vector<bool> > cornerFlag = getCorners(edge, corners);
		for (auto it = corners.begin(); it != corners.end(); it++){
			(*it) += offset;
		}
		display = new PointsDisplay(corners, 5);
		display->setDisplay("source", Size(0, 0), rgb, Scalar(255, 0, 0));
		display->doDisplay();
		delete display;

		PointSet polygon;	//记录多边形上的点
		// 寻找多边形的起始点
		Point startPoint = findStartPoint(edge);
		cout << "initial point = (" << startPoint.x << ", " << startPoint.y << ")" << endl;
		getPolygon(edge, startPoint, polygon);
		/*for (int i = 0; i < polygon.size(); i++){
			Point p = polygon[i];
			p.x += offsetX;
			p.y += offsetY;
			circle(rgb, p, 1, Scalar(0, 0, 0), CV_FILLED);
		}
		imshow("source", rgb); waitKey();*/

		// 源汉字取样时考虑目标汉字大小。 4.16
		Mat& strokeImgTarget = targetChar.strokeImage.at(no);
		Mat largerStrokeImgTarget(strokeImgTarget.size() + Size(10, 10), CV_8UC1, Scalar(0));
		for (int i = 0; i<strokeImgTarget.rows; i++){
			for (int j = 0; j<strokeImgTarget.cols; j++){
				largerStrokeImgTarget.at<uchar>(i + 5, j + 5) = 255 - strokeImgTarget.at<uchar>(i, j);
			}
		}
		Mat edgeTarget = getEdge(largerStrokeImgTarget);	// 边缘检测得到的矩阵，边缘为白色。
		PointSet polygonTarget;	//记录多边形上的点
		// 寻找多边形的起始点
		Point startPointTarget = findStartPoint(edgeTarget);
		getPolygon(edgeTarget, startPointTarget, polygonTarget);

		PointSet samplePoints = getSamplePoints(polygon, cornerFlag, polygonTarget.size());
		// Plot the polygon
		for (int i = 0; i < samplePoints.size(); i++){	// large stroke image -> character image
			samplePoints[i] += offset;
		}
		display = new PointsDisplay(samplePoints, 2, true);
		display->setDisplay("source", Size(0, 0), rgb);
		display->doDisplay();
		delete display;
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
	display = new MeshDisplay(triMesh, sourceCharVert);
	display->setDisplay("source", Size(0, 0), rgb, Scalar(255, 0, 0));
	display->doDisplay();
	delete display;
	TriMesh connectTri = getConnectTri(sourceCharVert, strokeEndAtVertex);
	cout << "There are " << connectTri.size() << " connection triangles in source character." << endl;
	triMesh.insert(triMesh.end(), connectTri.begin(), connectTri.end());
	cout << "There are " << triMesh.size() << " triangles in source character." << endl;
	display = new MeshDisplay(connectTri, sourceCharVert);
	display->setDisplay("source", Size(0, 0), rgb, Scalar(0, 255, 0));
	display->doDisplay();
	delete display;
}

void registerPointSetToTargetCharacter(){
	Mat rgb(targetChar.getCharSize() + Size(10, 10), CV_8UC3, Scalar(255, 255, 255));
	DisplayService *display = new CharacterDisplay(targetChar);
	display->setDisplay("target", Size(0, 0), rgb, Scalar(0, 0, 0));
	display->doDisplay();
	delete display;

	for (int no = 0; no < numStroke; no++){
		cout << "processing stroke " << no << endl;
		Point offset = targetChar.getStrokeOffset(no);
		Mat& strokeImg = targetChar.strokeImage.at(no);
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
			(*it) += offset;
		}
		display = new PointsDisplay(corners, 5);
		display->setDisplay("target", Size(0, 0), rgb, Scalar(255, 0, 0));
		display->doDisplay();
		delete display;

		// CPD registration
		vector<CPointDouble> X;
		vector<CPointDouble> Y;
		vector<int> S;
		for (int i = 0; i<edge.rows; i++){
			for (int j = 0; j<edge.cols; j++){
				if (edge.at<uchar>(i, j) > 128){
					CPointDouble cpdp(j + offset.x, i + offset.y);
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
		/*for (int i = 0; i<Y.size(); i++){
			circle(rgb, SGCPDPointToCVPoint(Y[i]), 5, Scalar(0, 255, 0));
		}*/

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
			keyPoints[i] -= offset;
		}
		vector< vector<int> > orderAtThisPoint = getPolygon(edge, keyPoints[0], polygon);	// 记录点在多边形上的顺序
		/*for (int i = 0; i < polygon.size(); i++){
			Point p = polygon[i];
			p.x += offsetX;
			p.y += offsetY;
			circle(rgb, p, 1, Scalar(0, 0, 0), CV_FILLED);
		}
		imshow("target", rgb); waitKey();*/

		PointSet samplePoints = getSamplePoints(polygon, cornerFlag, keyPoints, orderAtThisPoint);	// 采样后得到的点

		// Plot the polygon
		for (int i = 0; i < samplePoints.size(); i++){	// large stroke image -> character image
			samplePoints[i] += offset;
		}
		display = new PointsDisplay(samplePoints, 2, true);
		display->setDisplay("target", Size(0, 0), rgb);
		display->doDisplay();
		delete display;
		targetCharVert.insert(targetCharVert.end(), samplePoints.begin(), samplePoints.end());
	}
	cout << endl << "There are " << targetCharVert.size() << " points in target character." << endl;
	display = new MeshDisplay(triMesh, targetCharVert);
	display->setDisplay("target", Size(0, 0), rgb, Scalar(255, 0, 0));
	display->doDisplay();
	delete display;
}

int main( int, char** argv )
{
	readCharactersData();
	getTemplateFromSourceCharacter();
	registerPointSetToTargetCharacter();
	do{
		ARAPMorphing *morphing = new ARAPMorphing;
		morphing->setDisplay("morphing", sourceChar.getCharSize() + Size(40, 40));
		morphing->doMorphing(sourceCharVert, targetCharVert, triMesh, 10);
		morphing->doDisplay();
		delete morphing;
	} while (1);
	system("pause");
	return 0;
}