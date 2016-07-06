#pragma once

#include <vector>
#include "StdAfx.h"
#include "PointDouble.h"
using namespace std;

namespace SGCPD {
	class CPD {
	protected:
		/* internal data matrix for CPD */
		MatrixXd m_DataX;
		MatrixXd m_DataY;
		MatrixXd m_DataYStrokeInfo;
		MatrixXd m_DataTY;
		int m_nStrokeCount;
		vector<MatrixXd> m_DataYStrokes;
		vector<MatrixXd> m_DataTYStrokes;
		vector<MatrixXd> m_DataXStrokes;
		vector< vector<int> > m_LocalToGlobalMap;

		/* related to normalization and scaling */
		double m_dDataXTranslationX;
		double m_dDataXTranslationY;
		double m_dDataXScale;

		double m_dDataYTranslationX;
		double m_dDataYTranslationY;
		double m_dDataYScale;

	public:
		/* default constructor */
		CPD(void);

		/* constructor: initialize CPD data */
		CPD(vector<CPointDouble>& x, vector<CPointDouble>& y, vector<int>& s);

		/* set internal data */
		void SetData(vector<CPointDouble> x, vector<CPointDouble> y, vector<int> s);

		/* sparse input matrix */
		int SparseInput(int step);

		/* normalize input matrix */
		int NormalizeInput();

		/* denormalize input matrix */
		int DenormalizeInput();

		/* denormalize the output TY matrix */
		int DenormalizeOutput();

		/* CPD core method, virtual method, implementation leaves
		 * to inherited classes.
		 */
		virtual int CPDRegister(
			int majorIteration = 2,          // N/A for original CPD, global iteration count
			int minorIteration = 50,         // CPD/CPDLO iteration count
			double beta = 2,                 // the width of Gaussian kernel
			double lambda = 3,               // regularization weight 
			double tol = 1e-10,              // tolerance
			double outliers = 0.7,           // noise weight
			double sigma2 = 0                // sigma2
		) = 0;

		/* get stroke decomposition results */
		vector< vector<SGCPD::CPointDouble> > GetDecompositionResult();

		/* get data vector */
		vector<SGCPD::CPointDouble> GetDataPoints();

		/* get template vector */
		vector<SGCPD::CPointDouble> GetTemplatePoints();

		/* get original template vector */
		vector<SGCPD::CPointDouble> GetTemplatePointsOri();

		/* destructor */
		virtual ~CPD(void);

	protected:
		int GRBF(
			const MatrixXd& X,
			MatrixXd Y,
			MatrixXd& P,
			double beta,            
			double lambda,                  
			double maxIteration, 
			double tol, 
			double outliers, 
			double sigma2,
			bool isGlobal,
			int strokeIndex,        // if it is global, this parameter should be -1, else
			                        // should be stroke index.
			bool useLocalizedOperator,
			bool isOriginalCPD,
			bool useLastIterationP
		);
		
		MatrixXd CPD_G(
			const MatrixXd& x, 
			const MatrixXd& y, 
			double beta
		);

		void GetSigma2(
			const MatrixXd& X, 
			const MatrixXd& TY,
			const MatrixXd& Y,
			const MatrixXd& TX,
			const MatrixXd& P,
			double& sigma2,
			double& Np
		);

		double GetLikelihood(
			const MatrixXd& X,
			const MatrixXd& TY,
			const MatrixXd& Y,
			const MatrixXd& TX,
			const MatrixXd& P,
			const double& sigma2
		);

		MatrixXd CPD_P(
			const MatrixXd& X,
			const MatrixXd& Y,
			const double& sigma2,
			const double& outliers
		);

		void MatrixToPointVector(MatrixXd& matrix, vector<CPointDouble>& points);
		void PointVectorToMatrix(vector<CPointDouble>& points, MatrixXd& matrix);
	};
}
