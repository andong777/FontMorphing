#pragma once
#include "CCurvePoint.h"
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace GlyphVectorization {
	class CMyGlyph {
		public:
			std::vector<std::vector<cv::Point> > m_ContourPoints;	//轮廓点
			std::vector<std::vector<int> > m_ImportantPoints;						//被选中的轮廓点的下标
			std::vector<std::vector<CCurvePoint> > m_CurvePoints;	//曲线的端点
	};
}
