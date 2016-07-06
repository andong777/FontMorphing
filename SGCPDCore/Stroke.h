#pragma once
#include <vector>
#include "PointDouble.h"

namespace SGCPD {
	class Stroke {
	public:
		int m_nPointCount;
		vector<CPointDouble> m_Points;
	};
}
