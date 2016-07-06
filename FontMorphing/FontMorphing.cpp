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

float r = 1.;	// 不再需要了

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
		x1 /= r; x2 /= r; y1 /= r; y2 /= r;
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
			x1 /= r; x2 /= r; y1 /= r; y2 /= r;
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
		x1 /= r; x2 /= r; y1 /= r; y2 /= r;
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
			x1 /= r; x2 /= r; y1 /= r; y2 /= r;
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

#ifdef PARALLEL_MODE
#pragma omp parallel for num_threads(4)
#endif
	for(int no = 0; no < numStroke; no++){
		cout << "processing stroke "<<no<<endl;
		Mat& strokeImg = sourceChar.strokeImage.at(no);
		Mat largerStrokeImg(strokeImg.size() + Size(10, 10), CV_8UC1, Scalar(0));
		for(int i=0;i<strokeImg.rows;i++){
			for(int j=0;j<strokeImg.cols;j++){
				largerStrokeImg.at<uchar>(i+5, j+5) = 255 - strokeImg.at<uchar>(i, j);
			}
		}

		Point offset = sourceChar.getStrokeOffset(no);
		PointSet polygon;	//记录多边形上的点
		Mat edge = getEdge(largerStrokeImg, polygon);	// 边缘检测得到的矩阵，边缘为白色。
		/*for (int i = 0; i < polygon.size(); i++){
			Point p = polygon[i] + offset;
			circle(sourceCanvas, p, 1, Scalar(0, 0, 255), CV_FILLED);
			imshow("source", sourceCanvas); waitKey();
		}*/

		PointSet corners;
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
		display = new PointsDisplay(corners, 3);
		display->setDisplay(toScreen, "source", Size(0, 0), sourceCanvas, Scalar(255, 0, 0));
		display->doDisplay();
		delete display;
#ifdef VERBOSE
		imwrite(charName + "_corners.jpg", sourceCanvas);
#endif

		// Plot the polygon
		for (int i = 0; i < samplePoints.size(); i++){	// large stroke image -> character image
			samplePoints[i] += offset;
		}

		for (int i = 0; i < samplePoints.size(); i += kNumSample){
			circle(sourceCanvas, samplePoints[i], 4, Scalar(0, 0, 255), CV_FILLED);
		}
#ifdef VERBOSE
		imwrite("src_kp.jpg", sourceCanvas);
#endif

		display = new PointsDisplay(samplePoints, 2, true/*, true*/);
		display->setDisplay(toScreen, "source", Size(0, 0), sourceCanvas, Scalar(0, 255, 0));
		display->doDisplay();
		delete display;
		
		// Triangulation
		vector<Point2> inputPoints;
		for (auto it = samplePoints.begin(); it != samplePoints.end(); it++){
			inputPoints.push_back(CVPointToFade2DPoint(*it));
		}
		vector<Triangle2*> outputTriangles;
		Fade_2D dt;
		dt.insert(inputPoints);
		//dt.show("tri.ps");
		vector<Segment2> segments;
		//Mat m(Size(1000, 1000), CV_8UC3, Scalar::all(255));
		for (int i = 0; i < inputPoints.size(); i++){
			Segment2 seg(inputPoints[i], inputPoints[(i + 1)%inputPoints.size()]);
			segments.push_back(seg);
			//line(m, samplePoints[i] * 4, samplePoints[(i + 1) % samplePoints.size()] * 4, Scalar(0, 0, 255), 1);
		}
		//imshow("", m); waitKey();
		ConstraintGraph2* constraint = dt.createConstraint(segments, CIS_IGNORE_DELAUNAY);
		//assert(constraint->isPolygon());
		Zone2* zone = dt.createZone(constraint, ZL_INSIDE);
		dt.applyConstraintsAndZones();
		zone->getTriangles(outputTriangles);
		//dt.getTrianglePointers(outputTriangles);	// wrong
		
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
	imwrite("src.jpg", sourceCanvas);
#endif
	cout << endl << "There are " << sourceCharVert.size() << " points in source character." << endl;
	// get the connection triangles.
	display = new MeshDisplay(triMesh, sourceCharVert, false);
	display->setDisplay(toScreen, "source", Size(0, 0), sourceCanvas, Scalar(0, 255, 0));
	display->doDisplay();
	delete display;
	cout << "There are " << triMesh.size() << " triangles in source character." << endl;
#ifdef VERBOSE
	imwrite("src_tri.jpg", sourceCanvas);
#endif
}

void FontMorphing::registerPointSetToTargetCharacter(){
	targetCanvas = Mat(targetChar.getCharSize() + Size(10, 10), CV_8UC3, Scalar(255, 255, 255));
	DisplayService *display = new CharacterDisplay(targetChar);
	display->setDisplay(toScreen, "target", Size(0, 0), targetCanvas, Scalar(0, 0, 0));
	display->doDisplay();
	delete display;

	int numStroke = targetChar.getStrokeLength();

#ifdef PARALLEL_MODE
#pragma omp parallel for num_threads(4)
#endif
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
		/*for (int i = 0; i < polygon.size(); i++){
			Point p = polygon[i] + offset;
			circle(targetCanvas, p, 2, Scalar(0, 0, 255), CV_FILLED);
			imshow("target", targetCanvas); waitKey();
		}*/

		// find corner points on the contour
		PointSet corners;
		vector< vector<bool> > cornerFlag = getCornerPoints(edge, polygon, corners);
		PointSet corners2Display(corners.size());
		for (int i = 0; i < corners.size(); i++){
			corners2Display[i] = corners[i] + offset;
		}
		/*display = new PointsDisplay(corners2Display, 3);
		display->setDisplay(toScreen, "target", Size(0, 0), targetCanvas, Scalar(255, 0, 0));
		display->doDisplay();
		delete display;*/

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
#ifdef VERBOSE
		for (int i = 0; i < Y.size(); i++){
			circle(targetCanvas, SGCPDPointToCVPoint(Y[i]), 2, Scalar(255, 0, 0), CV_FILLED);
		}
		imshow("target", targetCanvas); waitKey();
#endif

		// find key points on the contour
		PointSet templatePointSet, dataPointSet;
		for (int i = 0; i < Y.size(); i++){
			templatePointSet.push_back(SGCPDPointToCVPoint(Y[i]));
		}
		for (int i = 0; i < X.size(); i++){
			dataPointSet.push_back(SGCPDPointToCVPoint(X[i]));
		}

		display = new PointsDisplay(templatePointSet,2, true);
		display->setDisplay(toScreen, "target", Size(0, 0), targetCanvas, Scalar(255, 0, 0));
		display->doDisplay();
		delete display;

		PointSet keyPoints;
		if (true /*sourceChar.strokeSkeleton.size() == sourceChar.getStrokeLength()*/) {
			Mat matTemp(targetChar.getCharSize() + Size(10, 10), CV_8UC1, Scalar::all(0));
			for (int i = 0; i < templatePointSet.size(); i++) {
				Point& p1 = templatePointSet[i];
				Point& p2 = templatePointSet[(i + 1) % templatePointSet.size()];
				line(matTemp, p1, p2, Scalar::all(255), 1);
			}
#ifdef VERBOSE
			//imshow("thinning", matTemp); waitKey();
#endif
			Mat filled = fillHoles(matTemp);
#ifdef VERBOSE
			//imshow("thinning", filled); waitKey();
#endif
			thinning(filled);
#ifdef VERBOSE
			//imshow("thinning", filled); waitKey();
#endif
			// 由点集注册后的模板点集，得到骨架线，进而获取骨架线段集合。	16.1.14
			PointPairSet templateSkeleton = getSkeleton(filled);

			// draw
			for (int i = 0; i < templateSkeleton.size(); i++) {
				auto seg = templateSkeleton[i];
				Point p1 = seg.first;
				Point p2 = seg.second;
				line(targetCanvas, p1, p2, Scalar(0, 255, 255), 1);
			}
			if (toScreen) {
				imshow("target", targetCanvas); waitKey();
			}

			keyPoints = findKeyPoints(templatePointSet, dataPointSet, templateSkeleton);
		}
		else {
			keyPoints = findKeyPoints(templatePointSet, dataPointSet);
		}
		
		display = new PointsDisplay(keyPoints, 4, true);
		display->setDisplay(toScreen, "target", Size(0, 0), targetCanvas, Scalar(0, 0, 255));
		display->doDisplay();
		delete display;

		for (int i = 0; i < keyPoints.size(); i++){
			keyPoints[i] -= offset;
		}

#ifdef VERBOSE
		imwrite("dst_kp.jpg", targetCanvas);
		/*for (int i = 0; i < keyPoints.size(); i++){
			Point p = keyPoints[i] + offset;
			cout << "(" << p.x << ", " << p.y << ")" << endl;
			circle(targetCanvas, p, 5, Scalar(0, 0, 255));
		}
		imshow("target", targetCanvas); waitKey();*/
#endif
		
		//PointSet& samplePoints = keyPoints;
		PointSet& samplePoints = getSamplePoints(polygon, keyPoints, kNumSample);
		useCornerPoints(polygon, corners, samplePoints);

		// Plot the polygon
		for (int i = 0; i < samplePoints.size(); i++){	// large stroke image -> character image
			samplePoints[i] += offset;
		}
		display = new PointsDisplay(samplePoints, 2, true);
		display->setDisplay(toScreen, "target", Size(0, 0), targetCanvas, Scalar(0, 255, 0));
		display->doDisplay();
		delete display;
		targetCharVert.insert(targetCharVert.end(), samplePoints.begin(), samplePoints.end());
	}
#ifdef VERBOSE
	imwrite("dst.jpg", targetCanvas);
#endif
	cout << endl << "There are " << targetCharVert.size() << " points in target character." << endl;
	display = new MeshDisplay(triMesh, targetCharVert, false);
	display->setDisplay(toScreen, "target", Size(0, 0), targetCanvas, Scalar(0, 255, 0));
	display->doDisplay();
	delete display;
#ifdef VERBOSE
	imwrite("dst_tri.jpg", targetCanvas);
#endif
}

bool FontMorphing::work(float ratio, bool toScreen)
{
	this->toScreen = toScreen;
	readCharactersData();
	assert(sourceChar.getStrokeLength() == targetChar.getStrokeLength());

	// 分辨率自适应
	double r = 1000 / sourceChar.charImage.size().width;
	kNumSample = int(kNumSampleStd * r);
	kNumSample = 15;
	cout << "Num sample set to " << kNumSample << "." << endl;

	getTemplateFromSourceCharacter();
	registerPointSetToTargetCharacter();
	connectTri = getConnectTri(sourceCharVert, targetCharVert, strokeEndAtVertex);
	cout << "There are " << connectTri.size() << " connection triangles in source character." << endl;
	triMesh.insert(triMesh.end(), connectTri.begin(), connectTri.end());
	DisplayService *display = new MeshDisplay(connectTri, sourceCharVert, false);
	display->setDisplay(toScreen, "source", Size(0, 0), sourceCanvas, Scalar(0, 255, 0));
	display->doDisplay();
	delete display;
	imwrite(outputCharDir + "\\" + charName + "_src_mesh.bmp", sourceCanvas);
	display = new MeshDisplay(connectTri, targetCharVert, false);
	display->setDisplay(toScreen, "target", Size(0, 0), targetCanvas, Scalar(0, 255, 0));
	display->doDisplay();
	delete display;
	imwrite(outputCharDir + "\\" + charName + "_dst_mesh.bmp", targetCanvas);
	assert(sourceCharVert.size() == targetCharVert.size());

	ARAPMorphing *morphing = new ARAPMorphing(charName);
	morphing->setMorphing(sourceCharVert, targetCharVert, triMesh, connectTri);
	Size imgSize = Size(max(sourceChar.getCharSize().width, targetChar.getCharSize().width) * 1.2, max(sourceChar.getCharSize().height, targetChar.getCharSize().height) * 1.2);
	//Size imgSize = Size(500, 500);
	morphing->setDisplay(toScreen, "morphing", imgSize);

	if (ratio < 1.f) {
		morphing->doMorphing(ratio, 0);	// 测试输出特定时刻
	} else {
		morphing->doMorphing(-1.f, (int)ratio);	// 测试输出过程
	}
	morphing->doDisplay();
	delete morphing;
	return true;
}