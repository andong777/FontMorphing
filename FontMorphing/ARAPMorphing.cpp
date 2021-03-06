#include "ARAPMorphing.h"
#include "MeshDisplay.h"
#include "Constant.h"
#include <iomanip>
#include <ctime>

using namespace std;
using namespace FM;

ARAPMorphing::ARAPMorphing(string charName)
{
	if (charName.length() > 0){
		this->charName = charName;
	}
	else{
		time_t t = std::time(nullptr);
		tm lt = *std::localtime(&t);
		ostringstream oss;
		oss << std::put_time(&lt, "%d-%m-%Y %H-%M-%S");
		this->charName = oss.str();
	}
}

ARAPMorphing::~ARAPMorphing()
{
}

void ARAPMorphing::setMorphing(PointSet& source, PointSet& target, TriMesh& mesh, TriMesh& connectTri)
{
	// 统一两个字形的大小
	int srcLeft = 10000, srcRight = 0, srcTop = 10000, srcBottom = 0, srcWidth, srcHeight;
	for (int i = 0; i < source.size(); i++){
		Point &p = source[i];
		if (p.x < srcLeft)	srcLeft = p.x;
		if (p.x > srcRight)	srcRight = p.x;
		if (p.y < srcTop)	srcTop = p.y;
		if (p.y > srcBottom)	srcBottom = p.y;
	}
	srcWidth = srcRight - srcLeft;
	srcHeight = srcBottom - srcTop;

	int dstLeft = 10000, dstRight = 0, dstTop = 10000, dstBottom = 0, dstWidth, dstHeight;
	for (int i = 0; i < target.size(); i++){
		Point &p = target[i];
		if (p.x < dstLeft)	dstLeft = p.x;
		if (p.x > dstRight)	dstRight = p.x;
		if (p.y < dstTop)	dstTop = p.y;
		if (p.y > dstBottom)	dstBottom = p.y;
	}
	dstWidth = dstRight - dstLeft;
	dstHeight = dstBottom - dstTop;

	for (int i = 0; i < source.size(); i++){
		float widthRatio = float(max(srcWidth, dstWidth)) / srcWidth;
		float heightRatio = float(max(srcHeight, dstHeight)) / srcHeight;
		Point &p = source[i];
		p.x = srcLeft + (p.x - srcLeft) * widthRatio;
		p.y = srcTop + (p.y - srcTop) * heightRatio;
	}
	for (int i = 0; i < target.size(); i++){
		float widthRatio = float(max(srcWidth, dstWidth)) / dstWidth;
		float heightRatio = float(max(srcHeight, dstHeight)) / dstHeight;
		Point &p = target[i];
		p.x = dstLeft + (p.x - dstLeft) * widthRatio;
		p.y = dstTop + (p.y - dstTop) * heightRatio;
	}

	MorphingService::setMorphing(source, target, mesh, connectTri);
}

void ARAPMorphing::doMorphing(float outputRatio, int numStep)
{
	// 固定点计算，使用最接近重心的点。 5.7
	Point center(0, 0);
	for (auto it = source.begin(); it != source.end(); it++){
		center += *it;
	}
	center.x /= (int)source.size();
	center.y /= (int)source.size();
	int minDist = infinity, minIdx = -1;
	for (int i = 0; i < source.size(); i++){
		Point diff = center - source[i];
		int dist = diff.x * diff.x + diff.y * diff.y;
		if (dist < minDist){
			minDist = dist;
			minIdx = i;
		}
	}
	int posFixed = minIdx;

	// todo: 这只是临时的丑陋做法，后面一定要改成从0开始的！
	// 准备数据。
	source.insert(source.begin(), Point());
	target.insert(target.begin(), Point());
	for (auto it = mesh.begin(); it != mesh.end(); it++){
		(*it).va++;
		(*it).vb++;
		(*it).vc++;
	}

	int numTri = mesh.size(), numVert = source.size()-1;
	Point *sv = new Point[numVert+1];
	Point *tv = new Point[numVert+1];
	for (int i = 1; i <= numVert; i++){
		sv[i] = source[i] - source[posFixed];
		tv[i] = target[i] - target[posFixed];
	}

	float *angle = new float[numTri];
	for (int i = 0; i < numTri; i++) angle[i] = .0;
	vector<MatrixXf> D;
	MatrixXf H = MatrixXf::Zero(2 * numVert, 2 * numVert);
	vector<MatrixXf> G_temp;
	for (int i = 0; i < numTri; i++){
		Triangle t = mesh[i];
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
   	} // end of numTri

	posFixed--;
	MatrixXf tempMatH(2 * numVert - 2, 2 * numVert - 2);
	int longEdge = 2 * posFixed, shortEdge = 2 * numVert - 2 * posFixed - 2;
	tempMatH.topLeftCorner(longEdge, longEdge) = H.topLeftCorner(longEdge, longEdge);
	tempMatH.bottomLeftCorner(shortEdge, longEdge) = H.bottomLeftCorner(shortEdge, longEdge);
	tempMatH.topRightCorner(longEdge, shortEdge) = H.topRightCorner(longEdge, shortEdge);
	tempMatH.bottomRightCorner(shortEdge, shortEdge) = H.bottomRightCorner(shortEdge, shortEdge);
	MatrixXf H_new = tempMatH.inverse();

	bool specified = outputRatio > 0 && outputRatio < 1;
	for (int i = 0; i <= (specified ? 0 : numStep); i++){
		float ratio = (specified ? outputRatio : 1.f * i / numStep);
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
		PointSet tempVert(numVert + 1);
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
			tempVert[j].x += source[posFixed].x + 20;
			tempVert[j].y += source[posFixed].y + 20;
		}
		tempVert.erase(tempVert.begin());
		pointSets.push_back(tempVert);
	} // end of numStep
	delete angle, tv, sv;

	// 恢复数据
	for (auto it = mesh.begin(); it != mesh.end(); it++){
		(*it).va--;
		(*it).vb--;
		(*it).vc--;
	}
	source.erase(source.begin());
	target.erase(target.begin());
}

void ARAPMorphing::doDisplay()
{
	// 为了去除形状插值算法中不知道怎么出现的空白部分。 5.7
	int leftMargin = 10;
	int topMargin = 10;
	for (int step = 0; step < pointSets.size(); step++){
		PointSet &points = pointSets[step];
		int leftBound = 10000, topBound = 10000;
		for (int i = 0; i < points.size(); i++){
			Point &p = points[i];
			if (p.x < leftBound)	leftBound = p.x;
			if (p.y < topBound)	topBound = p.y;
		}
		for (int i = 0; i < points.size(); i++){
			Point &p = points[i];
			p.x = leftMargin + p.x - leftBound;
			p.y = topMargin + p.y - topBound;
		}
	}

	mesh.erase(mesh.end() - connectTri.size(), mesh.end());	// 从三角网格中去除连接三角形
	for (int i = 0; i < pointSets.size(); i++){
		Mat originalCanvas = canvas.clone();
		/*if (toScreen){
			ostringstream osss;
			osss << "step: " << i << " / " << (pointSets.size() - 1);
			putText(canvas, osss.str(), Point(50, 30), CV_FONT_NORMAL, .5, Scalar(0, 0, 0));
		}*/
		ostringstream oss;
		oss << outputCharDir << "\\";
		DisplayService *plot;
		plot = new MeshDisplay(mesh, pointSets[i], false);
		plot->setDisplay(toScreen, "morphing", Size(0, 0), canvas, color);
		plot->doDisplay();
		delete plot;
		if (pointSets.size() > 1){
			oss << charName << "_" << (pointSets.size() - 1) << "_" << i << ".bmp";
		}
		else{	// 只输出一个特定的时刻
			oss << charName << ".bmp";
		}
		imwrite(oss.str(), canvas);

		canvas = originalCanvas.clone();
		plot = new MeshDisplay(mesh, pointSets[i], true);
		plot->setDisplay(false, "morphing", Size(0, 0), canvas, color);
		plot->doDisplay();
		delete plot;
		oss.str(""); oss.clear();
		oss << outputCharDir << "\\";
		if (pointSets.size() > 1){
			oss << charName << "_" << (pointSets.size() - 1) << "_" << i << "F.bmp";
		}
		else{	// 只输出一个特定的时刻
			oss << charName << "F.bmp";
		}
		imwrite(oss.str(), canvas);

		canvas = originalCanvas.clone();
	}
}

PointSet ARAPMorphing::getPointSetByStepIndex(int stepIndex)
{
	return pointSets.at(stepIndex);
}