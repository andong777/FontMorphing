#include "StdAfx.h"
#include "CPD.h"
#include "Stroke.h"
#include <algorithm>

namespace SGCPD {
	CPD::CPD(void) {
	}

	CPD::CPD(vector<CPointDouble>& x, vector<CPointDouble>& y, vector<int>& s) {
	}

	CPD::~CPD(void) {
	}

	int CPD::SparseInput(int step) {
		int rowCountX = m_DataX.rows();
		int rowCountY = m_DataY.rows();

		int sparseRowCountX = rowCountX / step;
		int sparseRowCountY = rowCountY / step;

		// random keep the points.
		int* pointIndexX = new int[rowCountX];
		int* pointIndexY = new int[rowCountY];
		for (int i = 0; i < rowCountX; i++) {
			pointIndexX[i] = i;
		}
		for (int i = 0; i < rowCountY; i++) {
			pointIndexY[i] = i;
		}
		MatrixXd tempX, tempY, tempYStroke;
		tempX.resize(sparseRowCountX, 2);
		tempY.resize(sparseRowCountY, 2);
		tempYStroke.resize(sparseRowCountY, 1);
		for (int i = 0; i < sparseRowCountX; i++) {
			int currentIndex = rand() % (rowCountX - i);
			tempX(i, 0) = m_DataX(pointIndexX[currentIndex], 0);
			tempX(i, 1) = m_DataX(pointIndexX[currentIndex], 1);
			int temp = pointIndexX[currentIndex];
			pointIndexX[currentIndex] = pointIndexX[rowCountX - 1 - i];
			pointIndexX[rowCountX - 1 - i] = temp;
		}
		for (int i = 0; i < sparseRowCountY; i++) {
			int currentIndex = rand() % (rowCountY - i);
			tempY(i, 0) = m_DataY(pointIndexY[currentIndex], 0);
			tempY(i, 1) = m_DataY(pointIndexY[currentIndex], 1);
			tempYStroke(i, 0) = m_DataYStrokeInfo(pointIndexY[currentIndex], 0);
			int temp = pointIndexY[currentIndex];
			pointIndexY[currentIndex] = pointIndexY[rowCountY - 1 - i];
			pointIndexY[rowCountY - 1 - i] = temp;
		}
		m_DataX.resize(sparseRowCountX, 2);
		m_DataY.resize(sparseRowCountY, 2);
		m_DataYStrokeInfo.resize(sparseRowCountY, 1);
		m_DataX = tempX;
		m_DataY = tempY;
		m_DataTY = m_DataY;
		m_DataYStrokeInfo = tempYStroke;
		delete[] pointIndexX;
		delete[] pointIndexY;
		return 0;
	}

	int CPD::NormalizeInput() {
		int n = m_DataX.rows();
		int m = m_DataY.rows();

		double sumXx = 0;
		double sumXy = 0;
		for (int i = 0; i < n; i++) {
			sumXx += m_DataX(i, 0);
			sumXy += m_DataX(i, 1);
		}
		double meanXx = (double)sumXx / n;
		double meanXy = (double)sumXy / n;
		double sumYx = 0;
		double sumYy = 0;
		for (int i = 0; i < m; i++) {
			sumYx += m_DataY(i, 0);
			sumYy += m_DataY(i, 1);
		}
		double meanYx = (double)sumYx / m;
		double meanYy = (double)sumYy / m;

		for (int i = 0; i < n; i++) {
			m_DataX(i, 0) -= meanXx;
			m_DataX(i, 1) -= meanXy;
		}
		for (int i = 0; i < m; i++) {
			m_DataY(i, 0) -= meanYx;
			m_DataY(i, 1) -= meanYy;
		}
		m_dDataXTranslationX = meanXx;
		m_dDataXTranslationY = meanXy;
		m_dDataYTranslationX = meanYx;
		m_dDataYTranslationY = meanYy;

		double squareSumX = 0.0;
		double squareSumY = 0.0;
		for (int i = 0; i < n; i++) {
			squareSumX += m_DataX(i, 0) * m_DataX(i, 0);
			squareSumX += m_DataX(i, 1) * m_DataX(i, 1);
		}
		for (int i = 0; i < m; i++) {
			squareSumY += m_DataY(i, 0) * m_DataY(i, 0);
			squareSumY += m_DataY(i, 1) * m_DataY(i, 1);
		}
		double scaleX = sqrt(squareSumX / n);
		double scaleY = sqrt(squareSumY / m);

		for (int i = 0; i < n; i++) {
			m_DataX(i, 0) /= scaleX;
			m_DataX(i, 1) /= scaleX;
		}
		for (int i = 0; i < m; i++) {
			m_DataY(i, 0) /= scaleY;
			m_DataY(i, 1) /= scaleY;
		}

		m_dDataXScale = scaleX;
		m_dDataYScale = scaleY;

		m_DataTY = m_DataY;

		return 0;
	}

	MatrixXd CPD::CPD_G(const MatrixXd& x, const MatrixXd& y, double beta) {
		MatrixXd G;
		double k = -2 * beta * beta;
		int n = x.rows();
		int m = y.rows();
		int d = 2; // MAY NEED UPDATE!!
		G = MatrixXd::Zero(n, m);
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < m; j++) {
				double temp = exp(((x(i, 0) - y(j, 0)) * (x(i, 0) - y(j, 0)) + 
					(x(i, 1) - y(j, 1)) * (x(i, 1) - y(j, 1))) / k);
				G(i, j) = temp;
			}
		}
		return G;
	}

	void CPD::GetSigma2(const MatrixXd& X, const MatrixXd& TY, const MatrixXd& Y, const MatrixXd& TX, const MatrixXd& P, double& sigma2, double& Np) {
		int N = X.rows();
		int D = 2;        // MAY NEED UPDATE!
		int M = Y.rows();

		MatrixXd E = MatrixXd::Zero(M, N);
		for (int i = 0; i < M; i++) {
			for (int j = 0; j < N; j++) {
				double sum = 0.0;
				sum = (X(j, 0) - TY(i, 0)) * (X(j, 0) - TY(i, 0)) + 
					(X(j, 1) - TY(i, 1)) * (X(j, 1) - TY(i, 1)) +
					(TX(j, 0) - Y(i, 0)) * (TX(j, 0) - Y(i, 0)) + 
					(TX(j, 1) - Y(i, 1)) * (TX(j, 1) - Y(i, 1));
				E(i, j) = sum / 2;
			}
		}
		Np = P.sum();
		MatrixXd temp = MatrixXd::Zero(M, N);
		for (int i = 0; i < M; i++) {
			for (int j = 0; j < N; j++) {
				temp(i, j) = E(i, j) * P(i, j);
			}
		}

		double lookat = temp.sum();
		sigma2 = temp.sum() / (Np * D);       
	}

	double CPD::GetLikelihood(const MatrixXd& X, const MatrixXd& TY, const MatrixXd& Y, const MatrixXd& TX, const MatrixXd& P, const double& sigma2) {
		int N = X.rows();
		int D = 2;        // MAY NEED UPDATE!
		int M = Y.rows();
		MatrixXd E = MatrixXd::Zero(M, N);
		for (int i = 0; i < M; i++) {
			for (int j = 0; j < N; j++) {
				double sum = 0.0;
				sum = (X(j, 0) - TY(i, 0)) * (X(j, 0) - TY(i, 0)) + 
					(X(j, 1) - TY(i, 1)) * (X(j, 1) - TY(i, 1)) +
					(TX(j, 0) - Y(i, 0)) * (TX(j, 0) - Y(i, 0)) + 
					(TX(j, 1) - Y(i, 1)) * (TX(j, 1) - Y(i, 1));
				E(i, j) = sum / 2;
			}
		}
		double Np = P.sum();
		double L;
		MatrixXd temp = MatrixXd::Zero(M, N);
		for (int i = 0; i < M; i++) {
			for (int j = 0; j < N; j++) {
				temp(i, j) = E(i, j) * P(i, j);
			}
		}
		L = temp.sum() / sigma2 + 0.5 * Np * D * log(sigma2);      
		return L;
	}

	MatrixXd CPD::CPD_P(const MatrixXd& X, const MatrixXd& Y, const double& sigma2, const double& outliers) {
		int N = X.rows();
		int M = Y.rows();
		MatrixXd P = MatrixXd::Zero(M, N);
		int D = 2;
		double diff, razn, outlierTmp, sp, pmn;

		double ksig = -2.0 * sigma2;
		outlierTmp = (outliers * M * pow(-ksig * 3.14159265358979, 0.5 * D)) / ((1 - outliers) * N);

		for (int n = 0; n < N; n++) {
			sp = 0;
			for (int m = 0; m < M; m++) {
				razn = 0;
				for (int d = 0; d < D; d++) {
					diff = X(n, d) - Y(m, d);
					diff = diff * diff;
					razn += diff;
				}
				pmn = exp(razn / ksig);
				P(m, n) = pmn;
				sp += pmn;
			}
			for (int m = 0; m < M; m++) {
				pmn = P(m, n);
				P(m, n) = pmn / (sp + outlierTmp);
			}
		}
		return P;
	}

	void CPD::SetData( vector<CPointDouble> x, vector<CPointDouble> y, vector<int> s ) {
		PointVectorToMatrix(x, m_DataX);
		PointVectorToMatrix(y, m_DataY);
		int m = s.size();
		m_DataYStrokeInfo.resize(m, 1);
		for (int i = 0; i < m; i++) {
			m_DataYStrokeInfo(i, 0) = s[i];
		}
		m_DataTY = m_DataY;

		// initialize the stroke specific data points of matrix Y.
		// in addition, get the local to global mapping.
		m_LocalToGlobalMap.resize(MAX_STROKE_COUNT);
		vector<Stroke> strokesOfDataY(MAX_STROKE_COUNT);   // !!!!!!!NOTICE THE MAX STROKE COUNT IS 100!
		for (int i = 0; i < MAX_STROKE_COUNT; i++) {
			strokesOfDataY[i].m_nPointCount = 0;
		}
		int rowsOfDataY = m_DataYStrokeInfo.rows();
		for (int i = 0; i < rowsOfDataY; i++) {
			int currentStrokeIndex =  m_DataYStrokeInfo(i);
			strokesOfDataY[currentStrokeIndex].m_nPointCount++;
			strokesOfDataY[currentStrokeIndex].m_Points.push_back(CPointDouble(m_DataTY(i, 0), m_DataTY(i, 1)));
			m_LocalToGlobalMap[currentStrokeIndex].push_back(i);
		}
		// add stroke specific info to doc->m_DataYStrokes and update doc->m_nStrokeCount.
		m_nStrokeCount = 0;
		for (int i = 0; i < MAX_STROKE_COUNT; i++) {
			if (strokesOfDataY[i].m_nPointCount != 0) {
				m_nStrokeCount++;
				MatrixXd currentStrokeMatrix;
				currentStrokeMatrix.resize(strokesOfDataY[i].m_Points.size(), 2);
				vector<CPointDouble>::iterator iter = strokesOfDataY[i].m_Points.begin();
				int currentPointIndex = 0;
				while (iter != strokesOfDataY[i].m_Points.end()) {
					currentStrokeMatrix(currentPointIndex, 0) = iter->x;
					currentStrokeMatrix(currentPointIndex, 1) = iter->y;
					currentPointIndex++;
					iter++;
				}
				m_DataYStrokes.push_back(currentStrokeMatrix);
			}
		}
		m_DataTYStrokes = m_DataYStrokes;
	}

	int CPD::GRBF(const MatrixXd& X, MatrixXd Y, MatrixXd& P, double beta, double lambda, double maxIteration, double tol, double outliers, double sigma2, bool isGlobal, int strokeIndex, /* if it is global, this parameter should be -1, else */ /* should be stroke index. */ bool useLocalizedOperator, bool isOriginalCPD, bool useLastIterationP) {
		int n = X.rows();
		if (n == 0) {
			return 0;
		}
		int m = Y.rows();
		MatrixXd TX = X;
		MatrixXd TY = Y;
		MatrixXd WX = MatrixXd::Zero(n, 2);
		MatrixXd WY = MatrixXd::Zero(m, 2);

		int iter = 0;
		double ntol = tol + 10;
		double L = 1.0;
		if (sigma2 == 0) {
			MatrixXd sumX = MatrixXd::Zero(1, 2);
			MatrixXd sumY = MatrixXd::Zero(2, 1);
			for (int i = 0 ; i < n; i++) {
				sumX(0, 0) += X(i, 0);
				sumX(0, 1) += X(i, 1);
			}
			for (int i = 0 ; i < m; i++) {
				sumY(0, 0) += Y(i, 0);
				sumY(1, 0) += Y(i, 1);
			}
			sigma2 = (m * (X.transpose() * X).trace() +
				n * (Y.transpose() * Y).trace() - 
				2 * (sumX * sumY)(0, 0)) / (m * n * 2);
		}

		double sigma2Init = sigma2;

		MatrixXd GY = CPD_G(Y, Y, beta);
		MatrixXd GX = CPD_G(X, X, beta);

		if (isGlobal && useLocalizedOperator) {
			// get localized restriction matrix.
			MatrixXd Loc = MatrixXd::Zero(m, m);
			for (int i = 0; i < m; i++) {
				for (int j = 0; j < m; j++) {
					if (m_DataYStrokeInfo(i) == m_DataYStrokeInfo(j)) {
						Loc(i, j) = 1;
					} else {
						Loc(i, j) = 0;
					}
				}
			}

			// get dot product of GY and Loc matrix.
			for (int i = 0; i < m; i++) {
				for (int j = 0; j < m; j++) {
					GY(i, j) *= Loc(i, j);
				}
			}
		}

		while ((iter < maxIteration) && (ntol > tol) && (sigma2 > 1e-8)) {
			double LOld = L;

			// E-step.
			MatrixXd PY = CPD_P(X, TY, sigma2, outliers);
			MatrixXd PX = CPD_P(Y, TX, sigma2, outliers);
			if (!useLastIterationP || iter != 0) {
				P = (PY + PX.transpose()) * 0.5;
			} 

			L = GetLikelihood(X, TY, Y, TX, P, sigma2);
			L = L + lambda / 2 * (WY.transpose() * GY * WY).trace() +
				lambda / 2 * (WX.transpose() * GX * WX).trace();

			ntol = abs((L - LOld) / L);

			// M-step.
			MatrixXd dPY = MatrixXd::Zero(m, m);
			for (int i = 0; i < m; i++) {
				for (int j = 0; j < n; j++) {
					dPY(i, i) += P(i, j);
				}
			}
			MatrixXd eyeM = MatrixXd::Zero(m, m);
			for (int i = 0; i < m; i++) {
				eyeM(i, i) = 1;
			}
			MatrixXd tempYLeft = dPY * GY + lambda * sigma2 * eyeM;
			MatrixXd tempYRight = P * X - dPY * Y;
			WY = tempYLeft.colPivHouseholderQr().solve(tempYRight);
			TY = Y + GY * WY;

			MatrixXd dPX = MatrixXd::Zero(n, n);
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < m; j++) {
					dPX(i, i) += P(j, i);
				}
			}

			MatrixXd eyeN = MatrixXd::Zero(n, n);
			for (int i = 0; i < n; i++) {
				eyeN(i, i) = 1;
			}
			MatrixXd tempXLeft = dPX * GX + lambda * sigma2 * eyeN;
			MatrixXd tempXRight = P.transpose() * Y - dPX * X;
			WX = tempXLeft.colPivHouseholderQr().solve(tempXRight);
			TX = X + GX * WX;

			double sigma2Save = sigma2;
			double Np;
			GetSigma2(X, TY, Y, TX, P, sigma2, Np);

			// the caller has launched a global CPD.
			if (isGlobal) {
				m_DataTY = TY;
			} 

			// the caller has launched a local.
			if (!isOriginalCPD && !isGlobal) { 
				for (int i = 0; i < TY.rows(); i++) {
					m_DataTY(m_LocalToGlobalMap[strokeIndex][i], 0) = TY(i, 0);
					m_DataTY(m_LocalToGlobalMap[strokeIndex][i], 1) = TY(i, 1);
				}
			}
			iter++;
		}

		return 0;
	}

	void CPD::MatrixToPointVector(MatrixXd& matrix, vector<CPointDouble>& points) {
		int m = matrix.rows();
		for (int i = 0; i < m; i++) {
			CPointDouble curPoint;
			curPoint.x = matrix(i, 0);
			curPoint.y = matrix(i, 1);
			points.push_back(curPoint);
		}
	}

	void CPD::PointVectorToMatrix(vector<CPointDouble>& points, MatrixXd& matrix) {
		int m = points.size();
		matrix.resize(m, 2);
		for (int i = 0; i < m; i++) {
			matrix(i, 0) = points[i].x;
			matrix(i, 1) = points[i].y;
		}
	}

	vector< vector<SGCPD::CPointDouble> > CPD::GetDecompositionResult() {
		vector< vector<SGCPD::CPointDouble> > result;
		for (int i = 0; i < m_DataXStrokes.size(); i++) {
			vector<SGCPD::CPointDouble> curResult;
			MatrixToPointVector(m_DataXStrokes[i], curResult);
			result.push_back(curResult);
		}
		return result;
	}

	vector<SGCPD::CPointDouble> CPD::GetDataPoints() {
		vector<SGCPD::CPointDouble> result;
		MatrixToPointVector(m_DataX, result);
		return result;
	}

	vector<SGCPD::CPointDouble> CPD::GetTemplatePoints() {
		vector<SGCPD::CPointDouble> result;
		MatrixToPointVector(m_DataTY, result);
		return result;
	}

	vector<SGCPD::CPointDouble> CPD::GetTemplatePointsOri() {
		vector<SGCPD::CPointDouble> result;
		MatrixToPointVector(m_DataY, result);
		return result;
	}

	int CPD::DenormalizeInput() {
		int n = m_DataX.rows();
		int m = m_DataY.rows();

		for (int i = 0; i < n; i++) {
			m_DataX(i, 0) *= m_dDataXScale;
			m_DataX(i, 1) *= m_dDataXScale;
		}
		for (int i = 0; i < m; i++) {
			m_DataY(i, 0) *= m_dDataYScale;
			m_DataY(i, 1) *= m_dDataYScale;
		}
		for (int i = 0; i < m_DataXStrokes.size(); i++) {
			int m = m_DataXStrokes[i].rows();
			for (int j = 0; j < m; j++) {
				m_DataXStrokes[i](j, 0) *= m_dDataXScale;
				m_DataXStrokes[i](j, 1) *= m_dDataXScale;
			}
		}

		for (int i = 0; i < n; i++) {
			m_DataX(i, 0) += m_dDataXTranslationX;
			m_DataX(i, 1) += m_dDataXTranslationY;
		}
		for (int i = 0; i < m; i++) {
			m_DataY(i, 0) += m_dDataYTranslationX;
			m_DataY(i, 1) += m_dDataYTranslationY;
		}
		for (int i = 0; i < m_DataXStrokes.size(); i++) {
			int m = m_DataXStrokes[i].rows();
			for (int j = 0; j < m; j++) {
				m_DataXStrokes[i](j, 0) += m_dDataXTranslationX;
				m_DataXStrokes[i](j, 1) += m_dDataXTranslationY;
			}
		}

		return 0;
	}

	int CPD::DenormalizeOutput() {
		int m = m_DataY.rows();

		for (int i = 0; i < m; i++) {
			m_DataTY(i, 0) *= m_dDataXScale;
			m_DataTY(i, 1) *= m_dDataXScale;
		}

		for (int i = 0; i < m; i++) {
			m_DataTY(i, 0) += m_dDataXTranslationX;
			m_DataTY(i, 1) += m_dDataXTranslationY;
		}
		return 0;
	}
}



