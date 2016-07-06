#pragma once
#include "CPD.h"

namespace SGCPD {
	class OriCPD : public CPD{
	private:
	public:
		OriCPD(void);

		/* constructor: initialize original CPD data */
		OriCPD(vector<CPointDouble> x, vector<CPointDouble> y, vector<int> s);

		virtual ~OriCPD(void);

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
		);
	};
}
