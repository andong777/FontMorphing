#include "StdAfx.h"
#include "SGCPD.h"
#include "Stroke.h"
#include "PointDouble.h"

namespace SGCPD {
	SGCPD::SGCPD(void) {
	}

	SGCPD::SGCPD( vector<CPointDouble>& x, vector<CPointDouble>& y, vector<int>& s ) : CPD(x, y, s){

	}

	SGCPD::~SGCPD(void) {
	}

	int SGCPD::CPDRegister(
		int majorIteration,          // N/A for original CPD, global iteration count
		int minorIteration,          // CPD/CPDLO iteration count
		double beta,                 // the width of Gaussian kernel
		double lambda,               // regularization weight 
		double tol,                  // tolerance
		double outliers,             // noise weight
		double sigma2)               // sigma2 
	{
		UpdateStrokeDataY();
		MatrixXd P;
		P.resize(m_DataX.rows(), m_DataY.rows());
		for (int iterCount = 0; iterCount < majorIteration; iterCount++) {
			// global CPD.
			vector< vector<int> > localToGlobalMap(MAX_STROKE_COUNT);
			bool useOriginalP;
			if (iterCount == 0) {
				useOriginalP = false;
			} else {
				useOriginalP = true;
			}
			GRBF(m_DataX, m_DataY, P, beta, lambda, minorIteration, tol, outliers, sigma2, true, -1, true, false, useOriginalP);

			// local CPD.
			UpdateStrokeDataX(P);

			vector<MatrixXd> tempDataXStrokes = m_DataXStrokes;
			for (int i = 0; i < m_nStrokeCount; i++) {
				//AfxMessageBox(_T("Local CPD."));
				MatrixXd PLocal;
				DeleteOutliers(tempDataXStrokes[i]);
				PLocal.resize(tempDataXStrokes[i].rows(), m_DataYStrokes[i].rows());
				GRBF(tempDataXStrokes[i], m_DataYStrokes[i], PLocal, beta, lambda, 30, tol, outliers, sigma2, false, i, true, false, false);
			}

			//---------------------------DELETED---------------------------------------
			// this statement is necessary and important because after deleting
			// the outliers, the m_DataXStrokes has been broken, we need to restore
			// it, it won't influence the next global iteration, because the local
			// iteration has done and what we need is the whole points registration
			// result.
			// UpdateStrokeDataX(P);
			//--------------------------------------------------------------------------
		}
		return 0;
	}

	void SGCPD::UpdateStrokeDataY() {
		// update the stroke specific data points of matrix Y and 
		// the local to global mapping.
		m_LocalToGlobalMap.clear();
		m_LocalToGlobalMap.resize(MAX_STROKE_COUNT);
		m_DataYStrokes.clear();

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

	void SGCPD::UpdateStrokeDataX(const MatrixXd& P) {
		// initialize the stroke specific data points of matrix X.
		vector<Stroke> strokesOfDataX(MAX_STROKE_COUNT);   // !!!!!!!NOTICE THE MAX STROKE COUNT IS 100!
		for (int i = 0; i < MAX_STROKE_COUNT; i++) {
			strokesOfDataX[i].m_nPointCount = 0;
		}
		int rowsOfDataX = m_DataX.rows();
		int rowsOfDataY = m_DataY.rows();
		m_DataXStrokes.clear();

		for (int i = 0; i < rowsOfDataX; i++) {
			int relatedDataYIndex = 0;
			double maxPmn = -1;
			for (int j = 0; j < rowsOfDataY; j++) {
				if (P(j, i) > maxPmn) {
					maxPmn = P(j, i);
					relatedDataYIndex = j;
				}
			}
			int currentStrokeIndex =  m_DataYStrokeInfo(relatedDataYIndex);
			strokesOfDataX[currentStrokeIndex].m_nPointCount++;
			strokesOfDataX[currentStrokeIndex].m_Points.push_back(CPointDouble(m_DataX(i, 0), m_DataX(i, 1)));
		}
		// add stroke specific info to doc->m_DataXStrokes.
		for (int i = 0; i < m_nStrokeCount; i++) {
			MatrixXd currentStrokeMatrix;
			currentStrokeMatrix.resize(strokesOfDataX[i].m_Points.size(), 2);
			vector<CPointDouble>::iterator iter = strokesOfDataX[i].m_Points.begin();
			int currentPointIndex = 0;
			while (iter != strokesOfDataX[i].m_Points.end()) {
				currentStrokeMatrix(currentPointIndex, 0) = iter->x;
				currentStrokeMatrix(currentPointIndex, 1) = iter->y;
				currentPointIndex++;
				iter++;
			}
			m_DataXStrokes.push_back(currentStrokeMatrix);
		}
	}

	void SGCPD::DeleteOutliers(vector<CPointDouble>& points) {
		int n = points.size();
		if (n == 0) {
			return;
		}
		vector< vector<CPointDouble> > data;
		for (int i = 0; i < n; i++) {
			vector<CPointDouble> curPointSet;
			curPointSet.push_back(points[i]);
			data.push_back(curPointSet);
		}
		while (true) {
			int size = data.size();
			bool continueMerge = false;
			for (int i = 0; i < size - 1; i++) {
				int minIndex;
				double minValue = INT_MAX;
				for (int j = i + 1; j < size; j++) {
					// get the minimum distance between set i and j.
					for (int a = 0; a < data[i].size(); a++) {
						for (int b = 0; b < data[j].size(); b++) {
							double curDis = (data[i][a].x - data[j][b].x) * (data[i][a].x - data[j][b].x) +
								(data[i][a].y - data[j][b].y) * (data[i][a].y - data[j][b].y);
							if (curDis < minValue) {
								minValue = curDis;
								minIndex = j;
							}
						}
					}
				}

				if (minValue <= DISTANCE_THRESHOLD) {
					continueMerge = true;

					// merge the set i and minIndex.
					for (int j = 0; j < data[minIndex].size(); j++) {
						data[i].push_back(data[minIndex][j]);
					}
					data.erase(data.begin() + minIndex);

					break;
				}
			}
			if (!continueMerge) {
				break;
			}
		}

		// get the maximal subset and assign it to the input reference.
		int size = data.size();
		int maxSize = 0;
		int maxIndex = 0;
		for (int i = 0; i < size; i++) {
			if (data[i].size() > maxSize) {
				maxSize = data[i].size();
				maxIndex = i;
			}
		}
		points = data[maxIndex];
	}


	/*
	 * This method is used for deleting the outliers in the matrix
	 * Notice the input matrix is the the target stroke point set,
	 * and the dimension of this matrix is M * 2.
	 */
	void SGCPD::DeleteOutliers(MatrixXd& matrix) {
		// convert the matrix to vector<CPointDouble>.
		vector<CPointDouble> points;
		MatrixToPointVector(matrix, points);

		// core delete outliers algorithm.
		DeleteOutliers(points);

		// convert the vector<CPointDouble> back to the matrix.
		PointVectorToMatrix(points, matrix);
	}
}

