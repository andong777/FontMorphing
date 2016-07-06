#pragma once
#include "CCurvePoint.h"
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace GlyphVectorization {
	class CMyGlyph {
		public:
			std::vector<std::vector<cv::Point> > m_ContourPoints;	//������
			std::vector<std::vector<int> > m_ImportantPoints;						//��ѡ�е���������±�
			std::vector<std::vector<CCurvePoint> > m_CurvePoints;	//���ߵĶ˵�
	};
}
