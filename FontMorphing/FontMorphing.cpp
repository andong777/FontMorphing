#include "FontMorphing.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include "OriCPD.h"
#include "Fade_2D.h"
#include "Utility.h"
#include "ARAPMorphing.h"
#include "PointsDisplay.h"
#include "MeshDisplay.h"
#include "CharacterDisplay.h"

FontMorphing::FontMorphing(string charName)
{
	this->charName = charName;
	this->toScreen = false;
}

FontMorphing::~FontMorphing()
{

}

void FontMorphing::readCharactersData(){
	string sourceCharPath = sourceCharDir + "\\" + charName;
	string targetCharPath = targetCharDir + "\\" + charName;
	sourceChar = CharacterImage();
	targetChar = CharacterImage();
	ifstream f;
	// show the character 1
	Mat sourceCharImage = imread(sourceCharPath + charImageSuffix);
	//imshow("source", sourceCharImage); waitKey();
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
		int i = 1;
		while (f >> x1 >> y1 >> x2 >> y2){
			ostringstream os;
			os << sourceCharPath << "_" << i << strokeImageSuffix;
			Mat strokeImg = imread(os.str(), CV_LOAD_IMAGE_GRAYSCALE);
			if (!strokeImg.data){	// 4.27
				ostringstream oss;
				oss << sourceCharPath << "_" << i << secondStrokeImageSuffix;
				strokeImg = imread(oss.str(), CV_LOAD_IMAGE_GRAYSCALE);
			}
			sourceChar.addStroke(strokeImg, Rect(Point(x1, y1), Point(x2, y2)));
			i++;
		}
		f.close();
	}
	cout << sourceChar.getStrokeLength() << " strokes in source" << endl;

	// show the character 2
	Mat targetCharImage = imread(targetCharPath + charImageSuffix);
	//imshow("target", targetCharImage); waitKey();
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
		int i = 1;
		while (f >> x1 >> y1 >> x2 >> y2){
			ostringstream os;
			os << targetCharPath << "_" << i << strokeImageSuffix;
			Mat strokeImg = imread(os.str(), CV_LOAD_IMAGE_GRAYSCALE);
			if (!strokeImg.data){
				ostringstream oss;
				oss << targetCharPath << "_" << i << secondStrokeImageSuffix;
				strokeImg = imread(oss.str(), CV_LOAD_IMAGE_GRAYSCALE);
			}
			targetChar.addStroke(strokeImg, Rect(Point(x1, y1), Point(x2, y2)));
			i++;
		}
		f.close();
	}
	cout << targetChar.getStrokeLength() << " strokes in target" << endl;
}

void FontMorphing::getTemplateFromSourceCharacter(){
	sourceCanvas = Mat(sourceChar.getCharSize() + Size(10, 10), CV_8UC3, Scalar(255, 255, 255));
	DisplayService *display = new CharacterDisplay(sourceChar);
	display->setDisplay(toScreen, "source", Size(0, 0), sourceCanvas, Scalar(0, 0, 0));
	display->doDisplay();
	delete display;

	int numStroke = sourceChar.getStrokeLength();
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

		PointSet polygon;	//记录多边形上的点
		Mat edge = getEdge(largerStrokeImg, polygon);	// 边缘检测得到的矩阵，边缘为白色。

		/*for (int i = 0; i < polygon.size(); i++){
			Point p = polygon[i];
			p.x += offsetX;
			p.y += offsetY;
			circle(sourceCanvas, p, 1, Scalar(0, 0, 0), CV_FILLED);
		}
		imshow("source", sourceCanvas); waitKey();*/

		PointSet corners;
		Point offset = sourceChar.getStrokeOffset(no);
		vector< vector<bool> > cornerFlag = getCornerPoints(edge, polygon, corners);

		// 源汉字取样时考虑目标汉字大小。 4.16
		Mat& strokeImgTarget = targetChar.strokeImage.at(no);
		Mat largerStrokeImgTarget(strokeImgTarget.size() + Size(10, 10), CV_8UC1, Scalar(0));
		for (int i = 0; i<strokeImgTarget.rows; i++){
			for (int j = 0; j<strokeImgTarget.cols; j++){
				largerStrokeImgTarget.at<uchar>(i + 5, j + 5) = 255 - strokeImgTarget.at<uchar>(i, j);
			}
		}
		PointSet polygonTarget;
		Mat edgeTarget = getEdge(largerStrokeImgTarget, polygonTarget);
		PointSet samplePoints = getSamplePoints(polygon, polygonTarget.size());
		useCornerPoints(polygon, corners, samplePoints);

		for (auto it = corners.begin(); it != corners.end(); it++){
			(*it) += offset;
		}
		display = new PointsDisplay(corners, 5);
		display->setDisplay(toScreen, "source", Size(0, 0), sourceCanvas, Scalar(255, 0, 0));
		display->doDisplay();
		delete display;

		// Plot the polygon
		for (int i = 0; i < samplePoints.size(); i++){	// large stroke image -> character image
			samplePoints[i] += offset;
		}
		display = new PointsDisplay(samplePoints, 2, true);
		display->setDisplay(toScreen, "source", Size(0, 0), sourceCanvas, Scalar(0, 0, 255));
		display->doDisplay();
		delete display;
		
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
		ConstraintGraph2* constraint = dt.createConstraint(segments, CIS_IGNORE_DELAUNAY);
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
#ifdef VERBOSE
	for (int i = 0; i < numStroke; i++){
		cout << "stroke " << i << " ends at vertex " << strokeEndAtVertex[i] << endl;
	}
#endif
	cout << endl << "There are " << sourceCharVert.size() << " points in source character." << endl;
	// get the connection triangles.
	display = new MeshDisplay(triMesh, sourceCharVert, false);
	display->setDisplay(toScreen, "source", Size(0, 0), sourceCanvas, Scalar(255, 0, 0));
	display->doDisplay();
	delete display;
	cout << "There are " << triMesh.size() << " triangles in source character." << endl;
}

void FontMorphing::registerPointSetToTargetCharacter(){
	targetCanvas = Mat(targetChar.getCharSize() + Size(10, 10), CV_8UC3, Scalar(255, 255, 255));
	DisplayService *display = new CharacterDisplay(targetChar);
	display->setDisplay(toScreen, "target", Size(0, 0), targetCanvas, Scalar(0, 0, 0));
	display->doDisplay();
	delete display;

	int numStroke = targetChar.getStrokeLength();
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
		PointSet polygon;	//记录多边形上的点
		Mat edge = getEdge(largerStrokeImg, polygon);

		// find corner points on the contour
		PointSet corners;
		vector< vector<bool> > cornerFlag = getCornerPoints(edge, polygon, corners);
		PointSet corners2Display(corners.size());
		for (int i = 0; i < corners.size(); i++){
			corners2Display[i] = corners[i] + offset;
		}
		display = new PointsDisplay(corners2Display, 5);
		display->setDisplay(toScreen, "target", Size(0, 0), targetCanvas, Scalar(255, 0, 0));
		display->doDisplay();
		delete display;

		// CPD registration
		vector<CPointDouble> X;
		vector<CPointDouble> Y;
		vector<int> S;
		for (auto it = polygon.begin(); it != polygon.end(); it++){
			X.push_back(CPointDouble((*it).x + offset.x, (*it).y + offset.y));
		}

#ifdef CPD_FROM_FILE
		ostringstream oss;
		oss << outputCharDir << "\\" << charName << "CPDY" << no << ".txt";
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
		cpd->CPDRegister(2, kCPDIterateTimes);
		cpd->DenormalizeOutput();
		Y = cpd->GetTemplatePoints();
		delete cpd;
		ostringstream oss;
		oss << outputCharDir << "\\" << charName << "CPDY" << no << ".txt";
		ofstream outf(oss.str(), ios::out);
		outf << Y.size() << endl;
		for(int i=0;i<Y.size();i++){
			outf << Y[i].x << " " << Y[i].y << endl;
		}
		outf.close();
#endif
		for (int i = 0; i<Y.size(); i++){
			circle(targetCanvas, SGCPDPointToCVPoint(Y[i]), 3, Scalar(0, 255, 0));
		}
		imshow("target", targetCanvas); waitKey();

		// find key points on the contour
		PointSet keyPoints;
		int *vertexBelongToTemplate = new int[X.size()];
		for (int i = 0; i<X.size(); i++)	vertexBelongToTemplate[i] = -1;
		for (int i = 0; i<Y.size(); i++){
			int minDist = infinity, index;
			for (int j = 0; j<X.size(); j++){
				if (vertexBelongToTemplate[j] < 0){	// 尚未确定该点的对应关系
					int dist = (X[j].x - Y[i].x)*(X[j].x - Y[i].x) + (X[j].y - Y[i].y)*(X[j].y - Y[i].y);
					if (dist < minDist){
						minDist = dist;
						index = j;
					}
				}
			}
			vertexBelongToTemplate[index] = i;
			keyPoints.push_back(polygon[index]);
		}
		delete[] vertexBelongToTemplate;
		cout << keyPoints.size() << " key points." << endl;

		/*for (int i = 0; i < keyPoints.size(); i++){
			Point p = keyPoints[i] + offset;
			cout << "(" << p.x << ", " << p.y << ")" << endl;
			circle(targetCanvas, p, 5, Scalar(0, 0, 255));
		}
		imshow("target", targetCanvas); waitKey();*/

		useCornerPoints(polygon, corners, keyPoints);
		PointSet& samplePoints = keyPoints;
		/*PointSet samplePoints = getSamplePoints(polygon, keyPoints);
		useCornerPoints(polygon, corners, samplePoints);*/

		// Plot the polygon
		for (int i = 0; i < samplePoints.size(); i++){	// large stroke image -> character image
			samplePoints[i] += offset;
		}
		display = new PointsDisplay(samplePoints, 2, true);
		display->setDisplay(toScreen, "target", Size(0, 0), targetCanvas, Scalar(0, 0, 255));
		display->doDisplay();
		delete display;
		targetCharVert.insert(targetCharVert.end(), samplePoints.begin(), samplePoints.end());
	}
	cout << endl << "There are " << targetCharVert.size() << " points in target character." << endl;
	display = new MeshDisplay(triMesh, targetCharVert, false);
	display->setDisplay(toScreen, "target", Size(0, 0), targetCanvas, Scalar(255, 0, 0));
	display->doDisplay();
	delete display;
	display = new MeshDisplay(connectTri, targetCharVert, false);
	display->setDisplay(toScreen, "target", Size(0, 0), targetCanvas, Scalar(0, 255, 0));
	display->doDisplay();
	delete display;
}

bool FontMorphing::work(bool toScreen)
{
	this->toScreen = toScreen;
	readCharactersData();
	assert(sourceChar.getStrokeLength() == targetChar.getStrokeLength());
	getTemplateFromSourceCharacter();
	registerPointSetToTargetCharacter();
	connectTri = getConnectTri(sourceCharVert, targetCharVert, strokeEndAtVertex);
	cout << "There are " << connectTri.size() << " connection triangles in source character." << endl;
	triMesh.insert(triMesh.end(), connectTri.begin(), connectTri.end());
	DisplayService *display = new MeshDisplay(connectTri, sourceCharVert, false);
	display->setDisplay(toScreen, "source", Size(0, 0), sourceCanvas, Scalar(0, 255, 0));
	display->doDisplay();
	delete display;
	imwrite(outputCharDir + "\\" + charName + "_src_mesh.jpg", sourceCanvas);
	display = new MeshDisplay(connectTri, targetCharVert, false);
	display->setDisplay(toScreen, "target", Size(0, 0), targetCanvas, Scalar(0, 255, 0));
	display->doDisplay();
	delete display;
	imwrite(outputCharDir + "\\" + charName + "_dst_mesh.jpg", targetCanvas);
	assert(sourceCharVert.size() == targetCharVert.size());
	for (int i = 0; i < sourceCharVert.size(); i++){
		sourceCharVert[i] += Point(100, 80);
		targetCharVert[i] += Point(100, 80);
	}
	ARAPMorphing *morphing = new ARAPMorphing(charName);
	morphing->setMorphing(sourceCharVert, targetCharVert, triMesh, connectTri);
	morphing->setDisplay(toScreen, "morphing", Size(max(sourceChar.getCharSize().width, targetChar.getCharSize().width) * 1.5, max(sourceChar.getCharSize().height, targetChar.getCharSize().height) * 1.5));
	//morphing->doMorphing(.5f, 0);	// 测试输出特定时刻
	morphing->doMorphing(-1.f, 5);	// 测试输出过程
	morphing->doDisplay();
	delete morphing;
	return true;
}