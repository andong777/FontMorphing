#pragma once

namespace SGCPD {
	class CPointDouble {
	public:
		double x;
		double y;
		CPointDouble(double x, double y) {
			this->x = x;
			this->y = y;
		}
		CPointDouble(){}
	};
}
