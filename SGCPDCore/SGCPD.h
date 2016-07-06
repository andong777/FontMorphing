#pragma once
#include "CPD.h"
#include "PointDouble.h"

namespace SGCPD {
	class SGCPD : public CPD{
	public:
		/* default constructor */
		SGCPD(void);

		/* constructor: initialize SGCPD data */
		SGCPD(vector<CPointDouble>& x, vector<CPointDouble>& y, vector<int>& s);

		/* set internal data */
		void SetData(vector<CPointDouble>& x, vector<CPointDouble>& y, vector<CPointDouble>& s);

		/* CPD core method */
		virtual int CPDRegister(
			int majorIteration = 2,          // N/A for original CPD, global iteration count
			int minorIteration = 50,         // CPD/CPDLO iteration count
			double beta = 2,                 // the width of Gaussian kernel
			double lambda = 3,               // regularization weight 
			double tol = 1e-10,              // tolerance
			double outliers = 0.7,           // noise weight
			double sigma2 = 0                // sigma2
		);

		/* destructor */
		virtual ~SGCPD(void);

	private:
		void UpdateStrokeDataY();
		void UpdateStrokeDataX(const MatrixXd& P);
		void DeleteOutliers(vector<CPointDouble>& points);
		void DeleteOutliers(MatrixXd& matrix);
	};
}
