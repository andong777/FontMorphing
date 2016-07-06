#include "StdAfx.h"
#include "OriCPD.h"
#include "PointDouble.h"
#include "Stroke.h"

namespace SGCPD {
	OriCPD::OriCPD(void) {
	}

	OriCPD::OriCPD( vector<CPointDouble> x, vector<CPointDouble> y, vector<int> s ) : CPD(x, y, s){

	}

	OriCPD::~OriCPD(void) {
	}

	int OriCPD::CPDRegister(
		int majorIteration,          // N/A for original CPD, global iteration count
		int minorIteration,          // CPD/CPDLO iteration count
		double beta,                 // the width of Gaussian kernel
		double lambda,               // regularization weight 
		double tol,                  // tolerance
		double outliers,             // noise weight
		double sigma2)               // sigma2 
	{
		MatrixXd P;
		P.resize(m_DataX.rows(), m_DataY.rows());
		GRBF(m_DataX, m_DataY, P, beta, lambda, minorIteration, tol, outliers, sigma2, true, -1, false, true, false);

		// initialize the stroke specific data points of matrix X.
		vector<Stroke> strokesOfDataX(MAX_STROKE_COUNT);   // !!!!!!!NOTICE THE MAX STROKE COUNT IS 100!
		for (int i = 0; i < MAX_STROKE_COUNT; i++) {
			strokesOfDataX[i].m_nPointCount = 0;
		}
		int rowsOfDataX = m_DataX.rows();
		int rowsOfDataY = m_DataYStrokeInfo.rows();

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

		// add stroke specific info to doc->m_DataXStrokesOri.
		for (int i = 0; i < MAX_STROKE_COUNT; i++) {
			if (strokesOfDataX[i].m_nPointCount != 0) {
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
		return 0;
	}
}
