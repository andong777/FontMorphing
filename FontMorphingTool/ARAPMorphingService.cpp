#include "ARAPMorphingService.h"
#include "MeshPlotDisplayService.h"

using namespace std;
using namespace FM;

#define SHOW_DETAILS

ARAPMorphingService::ARAPMorphingService()
{
}

ARAPMorphingService::~ARAPMorphingService()
{
}

void ARAPMorphingService::doMorphing(PointSet source, PointSet target, TriMesh mesh, int numStep)
{
	// todo: 这只是临时的丑陋做法，后面一定要改成从0开始的！
	// 准备数据。
	source.insert(source.begin(), Point());
	target.insert(target.begin(), Point());
	for (auto it = mesh.begin(); it != mesh.end(); it++){
		(*it).va++;
		(*it).vb++;
		(*it).vc++;
	}

	int posFixed = 10;
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
	this->source = source;
	this->target = target;
	this->mesh = mesh;
}

void ARAPMorphingService::doDisplay()
{
	for (int i = 0; i < pointSets.size(); i++){
		cout << "step: " << i << " / " << pointSets.size() - 1 << endl;
		Mat originalCanvas = canvas.clone();
		ostringstream osss;
		osss << "step: " << i << " / " << pointSets.size() - 1;
		putText(canvas, osss.str(), Point(250, 30), CV_FONT_NORMAL, .5, Scalar(0, 0, 0));
		DisplayService *plot = new MeshPlotDisplayService(mesh, pointSets[i]);
		plot->setDisplay("morphing", Size(0, 0), canvas, color);
		plot->doDisplay();
		delete plot;
		ostringstream oss;
		oss << i << ".jpg";
		imwrite(oss.str(), canvas);
		canvas = originalCanvas.clone();
	}
}

PointSet ARAPMorphingService::getPointSetByStepIndex(int stepIndex)
{
	return pointSets.at(stepIndex);
}